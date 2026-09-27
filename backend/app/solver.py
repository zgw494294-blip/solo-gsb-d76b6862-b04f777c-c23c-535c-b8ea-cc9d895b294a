"""齿轮系转速求解核心：校验 + 有理数精确高斯-若尔当消元 + 无解冲突定位。

关系类型：
  external  外啮合： za*wa + zb*wb = 0
  internal  内啮合： za*wa - zb*wb = 0
  planetary 行星组： zs*(ws-wc) + zr*(wr-wc) = 0

已知转速以“增广行” 1*wi = n 的形式进入增广矩阵 [A | b]。

每条通过校验的传动关系/已知转速都是一条可追溯条件，按“先关系、后已知
转速”的录入顺序编号（各自从 1 开始）。原模型无解时，从后往前逐条试删：
只有剩余条件仍无解才永久移除，最终得到确定的不可再缩减冲突集——集合
本身无解，删去其中任一条即有解（不保证条数最少）。判定全程使用有理数。

/api/gear-options（目标转速配齿）复用同一套录入校验：把指定关系的一个
齿数字段依次替换为 1~500 的整数逐一 RREF 求解，候选矛盾则舍弃；即使
整体欠定，只要目标轴转速不依赖自由变量且精确等于目标值即保留该齿数。

/api/torque-balance（理想传动转矩平衡）同样复用轴/关系校验（已知转速不
参与，故不校验）：按无损、无惯性的理想传动建模，关系系数矩阵 A 满足
Aω=0，对其零空间的任意允许运动 v，外加转矩向量 τ 均须满足 τ·v=0（功率
平衡 ⟺ τ 与零空间正交 ⟺ τ 在 A 的行空间中）。对 RREF 的每个自由列取零
空间基向量 v，平衡方程 Σvⱼτⱼ=0 中已知转矩移入常数项、未列出轴转矩为 0，
得到以待求轴转矩为未知量的线性系统，用有理数精确求解，区分无平衡解 /
唯一解 / 欠定；欠定时自由变量按轴录入顺序编号并给出各待求轴的精确仿射
表达式。

/api/transmission-paths（传动链路追踪）也只复用轴/关系校验（不使用转速
或转矩）：把每根轴看作图节点、每条传动关系看作一个独立的一步连接——
外/内啮合连接 a、b 两轴，行星组作为同一个关系（不拆成三条啮合）连接
s、r、c 三轴，组内任意两轴间一步可达；并行关系各自计数。以经过的关系
数为路径长度做无权 BFS，返回最短链路总数、按每步“关系录入序号、到达
轴录入序号”字典序最小的一条最短链路，以及所有最短链路必经的关系序号。

/api/mesh-harmonics（啮合谐波重合诊断）复用轴/关系/已知转速的全部校验与
同一套 RREF：外/内啮合基频 f=za·|ωa|/60 Hz，行星组基频 f=zs·|ωs−ωc|/60
Hz，全部有理数精确计算；模型无解时返回与 /api/solve 相同的不可再缩减
冲突集。欠定时基频仍依赖自由变量的关系标为“未确定”，不参与重合比较，
其余关系给出精确基频。对每条确定且非零的基频取 1~12 次谐波，按精确频率
分组，只列出包含至少两条不同关系的重合组；组按频率升序，组内成员按
（关系录入序号, 谐波次数）升序。

/api/brake-plans（最少制动轴方案）只复用轴/关系校验（不使用已知转速或
转矩）：关系齐次方程 Aω=0 的零空间即允许运动空间。对可制动轴的每个子集
B 追加 ω_j=0 的单位约束行后做有理数 RREF——目标轴在所有允许运动中转速
恒为零 ⟺ 目标轴为主元列且其主元行自由列系数全为 0（等价于单位行 e_t 已
落在行空间中）；保护轴仍能非零转动 ⟺ 保护轴是自由列或其主元行存在非零
自由列系数（e_p 不在行空间中）。枚举可制动轴的全部子集（含空集），返回
制动轴数最少的所有有效方案；方案内按轴录入顺序，方案间按该顺序的轴列表
字典序排列。
"""

from collections import deque
from fractions import Fraction
from itertools import combinations

from fastapi import HTTPException
from pydantic import BaseModel, ConfigDict

MIN_TEETH = 1
MAX_TEETH = 500
MIN_SHAFTS = 2
MAX_SHAFTS = 12
MAX_RELATIONS = 24
MIN_KNOWN = 1
MAX_KNOWN = 4
MIN_TORQUE_KNOWN = 1
MAX_TORQUE_KNOWN = MAX_SHAFTS    # 已知/待求互不相交，总数天然不超过轴数
MIN_TORQUE_UNKNOWN = 1
MAX_TORQUE_UNKNOWN = MAX_SHAFTS
MAX_DIGITS = 18  # 数字位数上限，防止恶意超大整数导致运算爆炸
MAX_HARMONIC = 12  # 啮合谐波重合诊断：每条确定且非零的基频取 1~12 次谐波


class Shaft(BaseModel):
    model_config = ConfigDict(extra="ignore")
    id: int | str | None = None


class Relation(BaseModel):
    model_config = ConfigDict(extra="ignore")
    type: str | None = None
    a: int | str | None = None
    b: int | str | None = None
    za: int | str | None = None
    zb: int | str | None = None
    s: int | str | None = None
    r: int | str | None = None
    c: int | str | None = None
    zs: int | str | None = None
    zr: int | str | None = None


class KnownSpeed(BaseModel):
    model_config = ConfigDict(extra="ignore")
    shaft: int | str | None = None
    speed: int | float | str | None = None


class SolveRequest(BaseModel):
    model_config = ConfigDict(extra="ignore")
    shafts: list[Shaft] | None = None
    relations: list[Relation] | None = None
    known: list[KnownSpeed] | None = None


class GearTarget(BaseModel):
    model_config = ConfigDict(extra="ignore")
    relation: int | str | None = None  # 关系序号（1 基）
    field: str | None = None           # za / zb / zs / zr
    shaft: int | str | None = None     # 目标轴编号
    speed: int | float | str | None = None  # 目标转速（有符号整数或分数，rpm）


class GearOptionsRequest(BaseModel):
    model_config = ConfigDict(extra="ignore")
    shafts: list[Shaft] | None = None
    relations: list[Relation] | None = None
    known: list[KnownSpeed] | None = None
    target: GearTarget | None = None


class KnownTorque(BaseModel):
    model_config = ConfigDict(extra="ignore")
    shaft: int | str | None = None
    torque: int | float | str | None = None  # 外加转矩（有符号整数或分数，N·m）


class TorqueBalanceRequest(BaseModel):
    model_config = ConfigDict(extra="ignore")
    shafts: list[Shaft] | None = None
    relations: list[Relation] | None = None
    known_torques: list[KnownTorque] | None = None   # 已知外加转矩（≥1 项）
    unknown_shafts: list[int | str | None] | None = None  # 待求转矩轴编号（≥1 根）


class TransmissionPathsRequest(BaseModel):
    model_config = ConfigDict(extra="ignore")
    shafts: list[Shaft] | None = None
    relations: list[Relation] | None = None
    start: int | str | None = None  # 起点轴编号
    end: int | str | None = None    # 终点轴编号


class BrakePlansRequest(BaseModel):
    model_config = ConfigDict(extra="ignore")
    shafts: list[Shaft] | None = None
    relations: list[Relation] | None = None
    target: int | str | None = None    # 目标轴（制动后须恒不转动）
    protected: int | str | None = None  # 保护轴（必须仍可转动）
    brakeable: list[int | str | None] | None = None  # 可制动轴（不含目标轴、保护轴）


def _parse_teeth(name: str, value: object, errors: list[str]) -> int | None:
    """齿数必须是 1~500 的整数（字符串 '30' 等规整写法也接受）。"""
    n = _parse_int(value)
    if n is None:
        errors.append(f"齿数 {name} 必须是 1~{MAX_TEETH} 的整数")
        return None
    if not (MIN_TEETH <= n <= MAX_TEETH):
        errors.append(f"齿数 {name}={n} 超出范围（{MIN_TEETH}~{MAX_TEETH}）")
        return None
    return n


def _parse_int(value: object) -> int | None:
    """接受 int 或无歧义的整数字符串（可带空白与正负号），拒绝布尔/小数/分数。"""
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        s = value.strip()
        if s and len(s.replace("-", "").replace("+", "")) <= MAX_DIGITS:
            try:
                return int(s)
            except ValueError:
                return None
    return None


def _parse_rational(value: object) -> Fraction | None:
    """解析有符号整数或分数字符串，如 120、-3/2、' 4 / -9 '。

    转速（rpm）与外加转矩（N·m）等带符号有理量共用本解析器。
    显式拒绝布尔值与 JSON 小数（1.5）——只接受整数或分数字符串。
    """
    if isinstance(value, bool) or isinstance(value, float):
        return None
    if isinstance(value, int):
        return Fraction(value)
    if isinstance(value, str):
        s = value.strip()
        if not s or "/" not in s:
            n = _parse_int(s) if s else None
            return Fraction(n) if n is not None else None
        parts = s.split("/")
        if len(parts) != 2:
            return None
        num, den = _parse_int(parts[0]), _parse_int(parts[1])
        if num is None or den is None or den == 0:
            return None
        return Fraction(num, den)
    return None


def _fraction_json(f: Fraction) -> dict:
    """分数的 JSON 表示。

    num/den 一律为十进制整数**字符串**：大整数若作为 JSON 数字返回，
    浏览器 JSON.parse 会按 IEEE 754 双精度舍入（如 7812500000000000000
    变为 7812500000000000512），导致页面分数失真。text 保留精确文本。
    """
    return {
        "num": str(f.numerator),
        "den": str(f.denominator),
        "text": str(f.numerator) if f.denominator == 1 else f"{f.numerator}/{f.denominator}",
    }


def _rref(mat: list[list[Fraction]], n: int) -> list[int]:
    """对增广矩阵 [A | b]（n 个未知量列 + 1 常数列）原地做高斯-若尔当消元。

    部分主元、列序即轴录入顺序；返回主元列下标列表。
    """
    m = len(mat)
    pivot_cols: list[int] = []
    lead = 0
    for col in range(n):
        if lead >= m:
            break
        pivot = next((r for r in range(lead, m) if mat[r][col] != 0), None)
        if pivot is None:
            continue
        mat[lead], mat[pivot] = mat[pivot], mat[lead]
        pv = mat[lead][col]
        mat[lead] = [v / pv for v in mat[lead]]
        for r in range(m):
            if r != lead and mat[r][col] != 0:
                factor = mat[r][col]
                mat[r] = [a - factor * b for a, b in zip(mat[r], mat[lead])]
        pivot_cols.append(col)
        lead += 1
    return pivot_cols


def _is_inconsistent(rows: list[list[Fraction]], n: int) -> bool:
    """有理数精确判定：方程组是否出现矛盾方程 0 = 非零常数。"""
    mat = [row[:] for row in rows]
    _rref(mat, n)
    return any(all(row[j] == 0 for j in range(n)) and row[n] != 0 for row in mat)


def _minimize_conflict(rows: list[list[Fraction]], n: int) -> list[int]:
    """删除法求不可再缩减的冲突集：按录入逆序逐条试删，剩余仍无解才永久移除。

    返回保留下来的条件下标（按录入顺序升序）。结果对给定顺序是确定的：
    集合本身无解，且删去其中任一条即有解；不保证条数最少。
    """
    kept = list(range(len(rows)))
    for i in reversed(range(len(rows))):
        trial = [j for j in kept if j != i]
        if _is_inconsistent([rows[j] for j in trial], n):
            kept = trial
    return kept


def _relation_row(
    rtype: str,
    positions: tuple[int, ...],
    teeth: tuple[int, ...],
    n: int,
) -> list[Fraction]:
    """按关系类型构造一条齐次传动方程的增广行（末列常数恒为 0）。

    录入校验与配齿枚举共用本函数，保证替换齿数后的候选行与正常录入
    所建行完全一致：
      external  za*wa + zb*wb = 0
      internal  za*wa - zb*wb = 0
      planetary zs*(ws-wc) + zr*(wr-wc) = 0
    """
    coeff = [Fraction(0)] * (n + 1)
    if rtype in ("external", "internal"):
        (ia, ib), (za, zb) = positions, teeth
        sign = 1 if rtype == "external" else -1
        coeff[ia] += Fraction(za)
        coeff[ib] += Fraction(sign * zb)
    else:
        (is_, ir, ic), (zs, zr) = positions, teeth
        coeff[is_] += Fraction(zs)
        coeff[ir] += Fraction(zr)
        coeff[ic] += Fraction(-(zs + zr))
    return coeff


def _prepare(req: SolveRequest, with_known: bool = True) -> dict:
    """校验录入并构建增广矩阵行（不做消元）。

    /api/solve、/api/gear-options 与 /api/torque-balance 共用同一套校验：
    任何一项不合法都在此集中抛出 HTTP 400。返回解析后的轴信息、增广行
    （每行为 n+1 个 Fraction）及每行的可追溯条件标签。
    with_known=False 时（转矩平衡）跳过已知转速的校验与建行——已知转速
    不参与转矩平衡计算，请求中也不必携带。
    """
    errors: list[str] = []

    # ---------- 轴 ----------
    if not req.shafts:
        raise HTTPException(status_code=400, detail={"errors": ["请至少录入 2 根轴"]})
    if not (MIN_SHAFTS <= len(req.shafts) <= MAX_SHAFTS):
        errors.append(f"轴的数量必须在 {MIN_SHAFTS}~{MAX_SHAFTS} 之间（当前 {len(req.shafts)}）")

    shaft_ids: list[int] = []
    seen: set[int] = set()
    for i, sh in enumerate(req.shafts):
        sid = _parse_int(sh.id)
        if sid is None:
            errors.append(f"第 {i + 1} 根轴编号非法：必须是唯一整数")
            continue
        shaft_ids.append(sid)
        if sid in seen:
            errors.append(f"轴编号重复：{sid}")
        seen.add(sid)

    index: dict[int, int] = {}
    for pos, sid in enumerate(shaft_ids):
        index[sid] = pos
    n = len(shaft_ids)

    # ---------- 传动关系 ----------
    if not req.relations:
        errors.append("请至少录入 1 条传动关系")
    elif len(req.relations) > MAX_RELATIONS:
        errors.append(f"传动关系最多 {MAX_RELATIONS} 条（当前 {len(req.relations)}）")

    rows: list[list[Fraction]] = []
    row_tags: list[tuple[str, int]] = []  # 每行对应的可追溯条件：(类型, 原始序号)
    for k, rel in enumerate(req.relations or []):
        rtype = rel.type
        prefix = f"关系 {k + 1}"
        if rtype not in ("external", "internal", "planetary"):
            errors.append(f"{prefix}：类型必须是外啮合/内啮合/行星组之一")
            continue

        def ref(sid: object, role: str) -> int | None:
            j = _parse_int(sid)
            if j is None:
                errors.append(f"{prefix}：{role}轴编号非法")
                return None
            if j not in index:
                errors.append(f"{prefix}：{role}轴 {j} 未在轴列表中定义")
                return None
            return index[j]

        if rtype in ("external", "internal"):
            ia, ib = ref(rel.a, "主动"), ref(rel.b, "从动")
            za = _parse_teeth("za", rel.za, errors)
            zb = _parse_teeth("zb", rel.zb, errors)
            if ia is not None and ib is not None and ia == ib:
                errors.append(f"{prefix}：外/内啮合的两根轴必须互异")
            if ia is not None and ib is not None and za is not None and zb is not None:
                rows.append(_relation_row(rtype, (ia, ib), (za, zb), n))
                row_tags.append(("relation", k + 1))
        else:
            is_ = ref(rel.s, "太阳轮")
            ir = ref(rel.r, "齿圈")
            ic = ref(rel.c, "行星架")
            zs = _parse_teeth("zs", rel.zs, errors)
            zr = _parse_teeth("zr", rel.zr, errors)
            roles = [x for x in (is_, ir, ic) if x is not None]
            if len(set(roles)) != len(roles):
                errors.append(f"{prefix}：太阳轮、齿圈、行星架三轴必须互异")
            if is_ is not None and ir is not None and ic is not None and zs is not None and zr is not None:
                rows.append(_relation_row(rtype, (is_, ir, ic), (zs, zr), n))
                row_tags.append(("relation", k + 1))

    # ---------- 已知转速（转矩平衡不校验、不建模） ----------
    if with_known:
        if not req.known:
            errors.append("请至少录入 1 个已知转速")
        elif not (MIN_KNOWN <= len(req.known) <= MAX_KNOWN):
            errors.append(f"已知转速数量必须在 {MIN_KNOWN}~{MAX_KNOWN} 之间（当前 {len(req.known)}）")

        known_shafts: set[int] = set()
        for k, ks in enumerate(req.known or []):
            sid = _parse_int(ks.shaft)
            if sid is None:
                errors.append(f"已知转速 {k + 1}：轴编号非法")
                continue
            if sid not in index:
                errors.append(f"已知转速 {k + 1}：轴 {sid} 未在轴列表中定义")
                continue
            if sid in known_shafts:
                errors.append(f"已知转速 {k + 1}：轴 {sid} 的转速重复给定")
                continue
            speed = _parse_rational(ks.speed)
            if speed is None:
                errors.append(
                    f"已知转速 {k + 1}（轴 {sid}）：转速 “{ks.speed}” 非法，"
                    "须为有符号整数或最简分数字符串（如 -120、3/2），分母不能为 0"
                )
                continue
            known_shafts.add(sid)
            row = [Fraction(0)] * (n + 1)
            row[index[sid]] = Fraction(1)
            row[n] = speed
            rows.append(row)
            row_tags.append(("known", k + 1))

    if errors:
        raise HTTPException(status_code=400, detail={"errors": errors})

    return {
        "shaft_ids": shaft_ids,
        "index": index,
        "n": n,
        "rows": rows,
        "row_tags": row_tags,
    }


def solve(req: SolveRequest) -> dict:
    prepared = _prepare(req)
    shaft_ids = prepared["shaft_ids"]
    n = prepared["n"]
    rows = prepared["rows"]
    row_tags = prepared["row_tags"]

    # ---------- 高斯-若尔当消元（RREF，部分主元，列序即轴录入顺序） ----------
    mat = [row[:] for row in rows]
    pivot_cols = _rref(mat, n)

    if any(all(row[j] == 0 for j in range(n)) and row[n] != 0 for row in mat):
        # 无解：删除法定位不可再缩减的冲突条件集（全程有理数判定）
        kept = _minimize_conflict(rows, n)
        return {
            "status": "inconsistent",
            "shafts": shaft_ids,
            "message": "方程组出现矛盾方程 0 = 非零常数，该齿轮系无解。",
            "conflict": [
                {"type": row_tags[j][0], "index": row_tags[j][1]} for j in kept
            ],
        }

    pivot_set = set(pivot_cols)
    free_cols = [j for j in range(n) if j not in pivot_set]
    rank = len(pivot_cols)

    pivot_row_of = {c: i for i, c in enumerate(pivot_cols)}

    if not free_cols:
        # 唯一解：RREF 主元行给出 x_c = b
        values: list[Fraction | None] = [None] * n
        for c, r in pivot_row_of.items():
            values[c] = mat[r][n]
        return {
            "status": "unique",
            "shafts": shaft_ids,
            "rank": rank,
            "free_count": 0,
            "values": [
                {"shaft": shaft_ids[j], "fraction": _fraction_json(values[j])}
                for j in range(n)
            ],
        }

    # 欠定：非主元轴为自由变量，主元轴按自由变量给仿射表达式
    free_pos = {c: t for t, c in enumerate(free_cols)}
    expressions = []
    for j in range(n):
        if j in free_pos:
            expressions.append(
                {
                    "shaft": shaft_ids[j],
                    "free": True,
                    "free_index": free_pos[j] + 1,
                    "constant": _fraction_json(Fraction(0)),
                    "terms": [],
                }
            )
        else:
            r = pivot_row_of[j]
            terms = [
                {"free_index": free_pos[c] + 1, "coef": _fraction_json(-mat[r][c])}
                for c in free_cols
                if mat[r][c] != 0
            ]
            expressions.append(
                {
                    "shaft": shaft_ids[j],
                    "free": False,
                    "constant": _fraction_json(mat[r][n]),
                    "terms": terms,
                }
            )
    return {
        "status": "underdetermined",
        "shafts": shaft_ids,
        "rank": rank,
        "free_count": len(free_cols),
        "free_shafts": [shaft_ids[c] for c in free_cols],
        "expressions": expressions,
    }


def torque_balance(req: TorqueBalanceRequest) -> dict:
    """理想传动转矩平衡：无损、无惯性，对零空间任意允许运动 v 有 τ·v = 0。

    轴与传动关系的校验和 /api/solve 完全复用（已知转速不参与，故不校验）。
    关系系数矩阵 A（m×n）的零空间即允许运动空间；外加转矩向量 τ 满足功率
    平衡 ⟺ τ 与零空间正交 ⟺ τ 在 A 的行空间中。对 RREF 的每个自由列 f 取
    基向量 v（v[f]=1，主元列 p 处 v[p]=−R[row(p)][f]），平衡方程 Σvⱼτⱼ=0
    只含待求轴转矩未知量：已知转矩移入常数项，未列出轴的转矩为 0。所得
    线性系统用有理数精确求解，区分无平衡解 / 唯一解 / 欠定；欠定时自由
    变量按轴录入顺序编号，各待求轴给出精确仿射表达式。
    """
    prepared = _prepare(req, with_known=False)
    shaft_ids = prepared["shaft_ids"]
    index = prepared["index"]
    n = prepared["n"]
    rel_rows = prepared["rows"]  # 仅传动关系行：已知转速不参与转矩平衡

    errors: list[str] = []

    # ---------- 已知外加转矩 ----------
    if not req.known_torques:
        errors.append("请至少录入 1 个已知外加转矩")
    elif len(req.known_torques) > MAX_TORQUE_KNOWN:
        errors.append(f"已知外加转矩最多 {MAX_TORQUE_KNOWN} 项（当前 {len(req.known_torques)}）")

    known_torque: dict[int, Fraction] = {}
    for k, kt in enumerate(req.known_torques or []):
        sid = _parse_int(kt.shaft)
        if sid is None:
            errors.append(f"已知转矩 {k + 1}：轴编号非法")
            continue
        if sid not in index:
            errors.append(f"已知转矩 {k + 1}：轴 {sid} 未在轴列表中定义")
            continue
        if sid in known_torque:
            errors.append(f"已知转矩 {k + 1}：轴 {sid} 的转矩重复给定")
            continue
        val = _parse_rational(kt.torque)
        if val is None:
            errors.append(
                f"已知转矩 {k + 1}（轴 {sid}）：转矩 “{kt.torque}” 非法，"
                "须为有符号整数或最简分数字符串（如 -120、3/2），分母不能为 0"
            )
            continue
        known_torque[sid] = val

    # ---------- 待求转矩轴 ----------
    if not req.unknown_shafts:
        errors.append("请至少选择 1 根待求转矩轴")
    elif len(req.unknown_shafts) > MAX_TORQUE_UNKNOWN:
        errors.append(f"待求转矩轴最多 {MAX_TORQUE_UNKNOWN} 根（当前 {len(req.unknown_shafts)}）")

    unknown_set: set[int] = set()
    for k, raw in enumerate(req.unknown_shafts or []):
        sid = _parse_int(raw)
        if sid is None:
            errors.append(f"待求轴 {k + 1}：轴编号非法")
            continue
        if sid not in index:
            errors.append(f"待求轴 {k + 1}：轴 {sid} 未在轴列表中定义")
            continue
        if sid in unknown_set:
            errors.append(f"待求轴 {k + 1}：轴 {sid} 重复选择")
            continue
        if sid in known_torque:
            errors.append(f"待求轴 {k + 1}：轴 {sid} 已录入已知外加转矩，同一轴不能同时已知和待求")
            continue
        unknown_set.add(sid)

    if errors:
        raise HTTPException(status_code=400, detail={"errors": errors})

    # 待求轴统一按轴录入顺序排列：自由变量编号与表达式顺序随之确定
    unknown_ids = sorted(unknown_set, key=lambda s: index[s])
    k = len(unknown_ids)

    base = {
        "shafts": shaft_ids,
        "known_torques": [
            {"shaft": sid, "torque": _fraction_json(known_torque[sid])}
            for sid in shaft_ids
            if sid in known_torque
        ],
        "unknown_shafts": unknown_ids,
        "zero_shafts": [
            sid for sid in shaft_ids
            if sid not in known_torque and sid not in unknown_set
        ],
    }

    # ---------- 关系系数矩阵 A 的零空间基（RREF 自由列） ----------
    mat = [row[:] for row in rel_rows]
    pivot_cols = _rref(mat, n)
    pivot_set = set(pivot_cols)
    free_cols = [j for j in range(n) if j not in pivot_set]
    pivot_row_of = {c: i for i, c in enumerate(pivot_cols)}
    nullity = len(free_cols)  # 零空间维数 = 独立平衡方程数

    # ---------- 组装平衡方程 M·u = b（u 为待求轴转矩，按轴录入顺序） ----------
    ucol_of = {index[sid]: t for t, sid in enumerate(unknown_ids)}
    sys_rows: list[list[Fraction]] = []
    for f in free_cols:
        # 基向量 v：v[f] = 1，主元列 p 处 v[p] = −R[row(p)][f]，其余为 0
        terms = [(f, Fraction(1))]
        terms += [(p, -mat[i][f]) for p, i in pivot_row_of.items() if mat[i][f] != 0]
        row = [Fraction(0)] * (k + 1)
        for j, c in terms:
            if j in ucol_of:
                row[ucol_of[j]] += c
            else:
                row[k] -= c * known_torque.get(shaft_ids[j], Fraction(0))
        sys_rows.append(row)

    # ---------- 有理数精确求解（与转速求解同一套 RREF） ----------
    tmat = [row[:] for row in sys_rows]
    tpivots = _rref(tmat, k)

    if any(all(row[j] == 0 for j in range(k)) and row[k] != 0 for row in tmat):
        return {
            **base,
            "status": "inconsistent",
            "nullity": nullity,
            "message": "平衡方程组出现矛盾方程 0 = 非零常数：外加转矩无法对零空间的"
                       "所有允许运动满足 τ·v = 0，该齿轮系无转矩平衡解。",
        }

    tpivot_set = set(tpivots)
    tfree = [j for j in range(k) if j not in tpivot_set]
    rank = len(tpivots)
    tpivot_row_of = {c: i for i, c in enumerate(tpivots)}

    if not tfree:
        # 唯一解：RREF 主元行给出 u_c = b
        values: list[Fraction] = [Fraction(0)] * k
        for c, r in tpivot_row_of.items():
            values[c] = tmat[r][k]
        return {
            **base,
            "status": "unique",
            "nullity": nullity,
            "rank": rank,
            "free_count": 0,
            "values": [
                {"shaft": unknown_ids[j], "fraction": _fraction_json(values[j])}
                for j in range(k)
            ],
        }

    # 欠定：非主元待求轴为自由变量，主元待求轴按自由变量给仿射表达式
    free_pos = {c: t for t, c in enumerate(tfree)}
    expressions = []
    for j in range(k):
        if j in free_pos:
            expressions.append(
                {
                    "shaft": unknown_ids[j],
                    "free": True,
                    "free_index": free_pos[j] + 1,
                    "constant": _fraction_json(Fraction(0)),
                    "terms": [],
                }
            )
        else:
            r = tpivot_row_of[j]
            expressions.append(
                {
                    "shaft": unknown_ids[j],
                    "free": False,
                    "constant": _fraction_json(tmat[r][k]),
                    "terms": [
                        {"free_index": free_pos[c] + 1, "coef": _fraction_json(-tmat[r][c])}
                        for c in tfree
                        if tmat[r][c] != 0
                    ],
                }
            )
    return {
        **base,
        "status": "underdetermined",
        "nullity": nullity,
        "rank": rank,
        "free_count": len(tfree),
        "free_shafts": [unknown_ids[c] for c in tfree],
        "expressions": expressions,
    }


_TYPE_CN = {"external": "外啮合", "internal": "内啮合", "planetary": "行星组"}


def _target_speed(
    rows: list[list[Fraction]],
    n: int,
    target_col: int,
    target: Fraction,
) -> Fraction | None:
    """对给定增广行做一次精确 RREF，判定目标轴转速是否为唯一确定值。

    返回该确定值（恰好等于 target 时）；方程组矛盾、目标轴是自由变量、或
    其转速仍依赖自由变量时返回 None（欠定但目标轴有确定常数值也算命中）。
    """
    mat = [row[:] for row in rows]
    pivot_cols = _rref(mat, n)

    # 矛盾方程 0 = 非零常数：候选模型无解，舍弃
    if any(all(row[j] == 0 for j in range(n)) and row[n] != 0 for row in mat):
        return None

    pivot_row_of = {c: i for i, c in enumerate(pivot_cols)}
    if target_col not in pivot_row_of:
        return None  # 目标轴本身是自由变量，转速不定
    r = pivot_row_of[target_col]
    free_cols = [j for j in range(n) if j not in pivot_row_of]
    # 目标轴所在主元行中任一自由变量系数非零 → 转速依赖自由变量
    if any(mat[r][c] != 0 for c in free_cols):
        return None
    value = mat[r][n]
    return value if value == target else None


def gear_options(req: GearOptionsRequest) -> dict:
    """目标转速配齿：枚举所选齿数字段 1~500，逐一有理数求解筛选。

    录入校验与 /api/solve 完全复用：先对原始模型整体走一遍 _prepare（轴/
    关系/已知转速的全部规则），再校验配齿目标参数。基础模型通过校验后，
    候选齿数 z∈1~500 必合法、其余录入不变，故每轮只通过同一
    _relation_row 重建目标关系的一条增广行，与重新校验建行完全等价。
    命中条件：目标轴转速不依赖任何自由变量（整体欠定亦可）且精确等于
    目标转速；候选矛盾则舍弃。
    """
    # 原始模型先过一遍统一校验（轴/关系/已知转速的全部规则）
    prepared = _prepare(req)
    shaft_ids = prepared["shaft_ids"]
    index = prepared["index"]
    n = prepared["n"]

    errors: list[str] = []
    tgt = req.target

    # ---------- 配齿目标参数 ----------
    if tgt is None:
        raise HTTPException(status_code=400, detail={"errors": ["缺少配齿目标参数 target"]})

    rel_count = len(req.relations or [])
    rel_pos: int | None = None
    rel: Relation | None = None
    rel_no = _parse_int(tgt.relation)
    if rel_no is None or not (1 <= rel_no <= rel_count):
        errors.append(f"配齿目标：关系序号必须在 1~{rel_count} 之间（当前 {tgt.relation}）")
    else:
        rel_pos = rel_no - 1
        rel = (req.relations or [])[rel_pos]

    field = (tgt.field or "").strip() if isinstance(tgt.field, str) else None
    field_ok = field in ("za", "zb", "zs", "zr")
    field_applies = False
    if not field_ok:
        errors.append("配齿目标：齿数字段必须是 za、zb、zs、zr 之一")
    elif rel is not None:
        if rel.type in ("external", "internal"):
            field_applies = field in ("za", "zb")
            if not field_applies:
                errors.append(f"关系 {rel_no} 是{_TYPE_CN.get(rel.type, '啮合')}，只能选择齿数 za 或 zb")
        else:
            field_applies = field in ("zs", "zr")
            if not field_applies:
                errors.append(f"关系 {rel_no} 是行星组，只能选择齿数 zs 或 zr")

    target_col: int | None = None
    target_sid = _parse_int(tgt.shaft)
    if target_sid is None:
        errors.append("配齿目标：目标轴编号非法")
    elif target_sid not in index:
        errors.append(f"配齿目标：目标轴 {target_sid} 未在轴列表中定义")
    else:
        target_col = index[target_sid]

    target_speed = _parse_rational(tgt.speed)
    if target_speed is None:
        errors.append(
            f"配齿目标：目标转速 “{tgt.speed}” 非法，"
            "须为有符号整数或分数字符串（如 -120、3/2），分母不能为 0"
        )

    original: int | None = None
    if rel is not None and field_applies:
        original = _parse_int(getattr(rel, field))
        if original is None:
            errors.append(f"配齿目标：所选字段 {field} 当前不是合法齿数，无法比较与原齿数的差距")

    if errors:
        raise HTTPException(status_code=400, detail={"errors": errors})

    assert rel is not None and rel_pos is not None and field is not None
    assert target_col is not None and target_speed is not None and original is not None

    # 基础模型已通过整套录入校验。候选齿数 z∈1~500 本身一定合法，其余录入
    # 原样保留，因此只需用同一 _relation_row 重建目标关系那一条增广行，
    # 与重新校验建行完全等价，但免去 500 次重复校验。
    base_rows = prepared["rows"]
    try:
        target_row_idx = next(
            i for i, tag in enumerate(prepared["row_tags"])
            if tag == ("relation", rel_no)
        )
    except StopIteration:  # 理论不可达：通过校验的关系必有对应行
        raise HTTPException(status_code=400, detail={"errors": [f"关系 {rel_no} 未参与建模"]})

    def row_with_teeth(z: int) -> list[Fraction]:
        if rel.type in ("external", "internal"):
            ia, ib = index[_parse_int(rel.a)], index[_parse_int(rel.b)]
            za, zb = (z, _parse_int(rel.zb)) if field == "za" else (_parse_int(rel.za), z)
            return _relation_row(rel.type, (ia, ib), (za, zb), n)
        is_ = index[_parse_int(rel.s)]
        ir = index[_parse_int(rel.r)]
        ic = index[_parse_int(rel.c)]
        zs, zr = (z, _parse_int(rel.zr)) if field == "zs" else (_parse_int(rel.zs), z)
        return _relation_row(rel.type, (is_, ir, ic), (zs, zr), n)

    # ---------- 枚举齿数 1~500：每轮只替换一个字段，其余录入保持不变 ----------
    matches: list[dict] = []
    for z in range(MIN_TEETH, MAX_TEETH + 1):
        cand_rows = list(base_rows)
        cand_rows[target_row_idx] = row_with_teeth(z)
        value = _target_speed(cand_rows, n, target_col, target_speed)
        if value is None:
            continue
        matches.append(
            {
                "teeth": z,
                "distance": abs(z - original),
                "speed": _fraction_json(value),
            }
        )

    # 按与原齿数差的绝对值、齿数依次升序
    matches.sort(key=lambda m: (m["distance"], m["teeth"]))

    return {
        "status": "ok",
        "shafts": shaft_ids,
        "relation": rel_no,
        "field": field,
        "target_shaft": target_sid,
        "target_speed": _fraction_json(target_speed),
        "original_teeth": original,
        "count": len(matches),
        "matches": matches,
    }


_ROLE_LABEL = {
    "a": "主动轴 a",
    "b": "从动轴 b",
    "s": "太阳轮 s",
    "r": "齿圈 r",
    "c": "行星架 c",
}


def _bfs_count(adj: list[list[tuple[int, int]]], source: int, n: int) -> tuple[list[int | None], list[int]]:
    """无权图 BFS：返回各节点最短路距离与最短路条数（并行关系分别计数）。

    adj[u] 的元素为 (关系录入序号, 邻接轴位置)；同一对轴之间的每条关系都是
    独立的一条边，各自贡献一条最短路。dist 为 None 表示不可达。
    """
    dist: list[int | None] = [None] * n
    count = [0] * n
    dist[source] = 0
    count[source] = 1
    queue = deque([source])
    while queue:
        u = queue.popleft()
        for _, v in adj[u]:
            if dist[v] is None:
                dist[v] = dist[u] + 1
                queue.append(v)
            if dist[v] == dist[u] + 1:
                count[v] += count[u]
    return dist, count


def transmission_paths(req: TransmissionPathsRequest) -> dict:
    """传动链路追踪：以经过的关系数为路径长度，求最短链路。

    轴与传动关系的校验和 /api/solve 完全复用（已知转速、转矩均不参与，故
    不校验）。图模型：每根轴是一个节点，每条关系是独立的一步连接——
    外/内啮合连接 a、b 两轴；行星组是同一个关系，连接 s、r、c 三轴，组内
    任意两轴一步可达（不拆成三条独立啮合）；同一对轴间的并行关系各自计数。

    以经过的关系数为路径长度做 BFS：返回最短链路总数（按关系序列区分，
    并行关系分别计数）、按每步“关系录入序号、到达轴录入序号”字典序最小
    的一条最短链路（逐步写明关系类型、两端角色与齿数），以及所有最短链路
    必经的关系序号。起终点同轴时返回一条长度为零的空链路；不连通时返回
    零条并说明。
    """
    prepared = _prepare(req, with_known=False)
    shaft_ids = prepared["shaft_ids"]
    index = prepared["index"]
    n = prepared["n"]

    # ---------- 起点 / 终点 ----------
    errors: list[str] = []
    start = _parse_int(req.start)
    if start is None:
        errors.append("起点轴编号非法")
    elif start not in index:
        errors.append(f"起点轴 {start} 未在轴列表中定义")
    end = _parse_int(req.end)
    if end is None:
        errors.append("终点轴编号非法")
    elif end not in index:
        errors.append(f"终点轴 {end} 未在轴列表中定义")
    if errors:
        raise HTTPException(status_code=400, detail={"errors": errors})

    # ---------- 建图：每条关系是一个独立的一步连接 ----------
    # adj[u] = [(关系录入序号 k, 邻接轴位置 v), ...]；rel_ends[k-1] 给出该关系
    # 各轴位置对应的（角色, 齿数），用于逐步写明两端角色与齿数。
    adj: list[list[tuple[int, int]]] = [[] for _ in range(n)]
    rel_types: list[str] = []
    rel_ends: list[dict[int, tuple[str, int | None]]] = []
    rel_teeth: list[dict[str, int]] = []
    for k, rel in enumerate(req.relations or [], start=1):
        rel_types.append(rel.type)
        if rel.type in ("external", "internal"):
            ia, ib = index[_parse_int(rel.a)], index[_parse_int(rel.b)]
            za, zb = _parse_int(rel.za), _parse_int(rel.zb)
            adj[ia].append((k, ib))
            adj[ib].append((k, ia))
            rel_ends.append({ia: ("a", za), ib: ("b", zb)})
            rel_teeth.append({"za": za, "zb": zb})
        else:
            # 行星组：s/r/c 三轴同属一条关系，组内任意两轴一步可达
            is_ = index[_parse_int(rel.s)]
            ir = index[_parse_int(rel.r)]
            ic = index[_parse_int(rel.c)]
            zs, zr = _parse_int(rel.zs), _parse_int(rel.zr)
            members = [(is_, "s", zs), (ir, "r", zr), (ic, "c", None)]
            for u, _, _ in members:
                for v, _, _ in members:
                    if u != v:
                        adj[u].append((k, v))
            rel_ends.append({is_: ("s", zs), ir: ("r", zr), ic: ("c", None)})
            rel_teeth.append({"zs": zs, "zr": zr})

    s_pos, t_pos = index[start], index[end]
    base = {"shafts": shaft_ids, "start": start, "end": end}

    # ---------- 同轴：一条长度为零的空链路 ----------
    if s_pos == t_pos:
        return {
            **base,
            "status": "ok",
            "length": 0,
            "count": 1,
            "path": [],
            "essential": [],
            "message": f"起点与终点同为轴 {start}：存在一条长度为零的空链路（不经过任何传动关系）。",
        }

    # ---------- 最短链路：正向 BFS 计数 + 反向 BFS 定位必经关系 ----------
    dist, count = _bfs_count(adj, s_pos, n)
    if dist[t_pos] is None:
        return {
            **base,
            "status": "disconnected",
            "length": None,
            "count": 0,
            "path": [],
            "essential": [],
            "message": f"轴 {start} 与轴 {end} 之间不存在传动链路：二者不在同一连通分量，最短链路总数为 0。",
        }

    length = dist[t_pos]
    total = count[t_pos]
    dist_t, count_t = _bfs_count(adj, t_pos, n)

    # 必经关系：被全部最短链路使用的关系。一条最短链路经过的轴位置严格沿
    # dist 递增，故每条关系在一条链路中至多出现一次；关系 k 的使用次数即
    # 经过它的最短链路条数，等于总数时即为必经。
    usage: dict[int, int] = {}
    for u in range(n):
        if dist[u] is None or dist_t[u] is None or dist[u] + dist_t[u] != length:
            continue
        for k, v in adj[u]:
            if dist[v] == dist[u] + 1 and dist_t[v] is not None and dist[v] + dist_t[v] == length:
                usage[k] = usage.get(k, 0) + count[u] * count_t[v]
    essential = sorted(k for k, c in usage.items() if c == total)

    # ---------- 字典序最小链路：逐步取最小的（关系录入序号, 到达轴录入序号） ----------
    steps = []
    u = s_pos
    for d in range(length):
        best: tuple[tuple[int, int], int, int] | None = None
        for k, v in adj[u]:
            if dist[v] == d + 1 and dist_t[v] is not None and dist[v] + dist_t[v] == length:
                key = (k, v)  # v 为 0 基轴位置，顺序与 1 基“到达轴录入序号”一致
                if best is None or key < best[0]:
                    best = (key, k, v)
        _, k, v = best
        fr_role, fr_teeth = rel_ends[k - 1][u]
        to_role, to_teeth = rel_ends[k - 1][v]
        steps.append(
            {
                "step": d + 1,
                "relation": k,
                "type": rel_types[k - 1],
                "from": {
                    "shaft": shaft_ids[u],
                    "role": fr_role,
                    "role_label": _ROLE_LABEL[fr_role],
                    "teeth": fr_teeth,
                },
                "to": {
                    "shaft": shaft_ids[v],
                    "role": to_role,
                    "role_label": _ROLE_LABEL[to_role],
                    "teeth": to_teeth,
                },
                "teeth": rel_teeth[k - 1],
            }
        )
        u = v

    return {
        **base,
        "status": "ok",
        "length": length,
        "count": total,
        "path": steps,
        "essential": essential,
    }


def mesh_harmonics(req: SolveRequest) -> dict:
    """啮合谐波重合诊断：各关系基频取 1~12 次谐波，按精确频率找跨关系重合。

    轴、传动关系与已知转速的校验和 /api/solve 完全复用；模型无解时返回与
    /api/solve 相同的不可再缩减冲突集。基频（全部有理数精确计算）：
      external/internal  f = za·|ωa|/60 Hz
      planetary          f = zs·|ωs−ωc|/60 Hz
    欠定时，基频表达式仍含自由变量的关系标为“未确定”，不参与重合比较；其余
    关系给出精确基频。对每条确定且非零的基频取 1~12 次谐波，按精确频率分组，
    只保留包含至少两条不同关系的重合组；组按频率升序，组内成员按
    （关系录入序号, 谐波次数）升序。
    """
    prepared = _prepare(req)
    shaft_ids = prepared["shaft_ids"]
    index = prepared["index"]
    n = prepared["n"]
    rows = prepared["rows"]
    row_tags = prepared["row_tags"]

    # ---------- 高斯-若尔当消元（与 /api/solve 同一套 RREF） ----------
    mat = [row[:] for row in rows]
    pivot_cols = _rref(mat, n)

    if any(all(row[j] == 0 for j in range(n)) and row[n] != 0 for row in mat):
        # 无解：删除法定位不可再缩减的冲突条件集（与 /api/solve 相同）
        kept = _minimize_conflict(rows, n)
        return {
            "status": "inconsistent",
            "shafts": shaft_ids,
            "message": "方程组出现矛盾方程 0 = 非零常数，该齿轮系无解。",
            "conflict": [
                {"type": row_tags[j][0], "index": row_tags[j][1]} for j in kept
            ],
        }

    pivot_row_of = {c: i for i, c in enumerate(pivot_cols)}
    free_cols = [j for j in range(n) if j not in pivot_row_of]

    def omega_expr(col: int) -> tuple[Fraction, dict[int, Fraction]]:
        """轴 col 转速的仿射表达式：常数项 + 各自由变量系数（自由轴自身即 t）。"""
        if col in pivot_row_of:
            r = pivot_row_of[col]
            terms = {c: -mat[r][c] for c in free_cols if mat[r][c] != 0}
            return mat[r][n], terms
        return Fraction(0), {col: Fraction(1)}

    # ---------- 各关系基频：依赖自由变量的标为未确定，不参与比较 ----------
    relations_out: list[dict] = []
    base_freqs: list[tuple[int, Fraction]] = []  # (关系录入序号, 确定且非零的基频)
    for k, rel in enumerate(req.relations or [], start=1):
        if rel.type in ("external", "internal"):
            teeth = _parse_int(rel.za)
            const, terms = omega_expr(index[_parse_int(rel.a)])
        else:
            teeth = _parse_int(rel.zs)
            s_const, s_terms = omega_expr(index[_parse_int(rel.s)])
            c_const, c_terms = omega_expr(index[_parse_int(rel.c)])
            const = s_const - c_const
            terms = dict(s_terms)
            for c, coef in c_terms.items():
                terms[c] = terms.get(c, Fraction(0)) - coef
                if terms[c] == 0:
                    del terms[c]
        if terms:
            # 基频仍依赖自由变量：未确定，不参与重合比较
            relations_out.append({"relation": k, "type": rel.type, "determined": False})
            continue
        freq = Fraction(teeth) * abs(const) / 60
        relations_out.append(
            {
                "relation": k,
                "type": rel.type,
                "determined": True,
                "frequency": _fraction_json(freq),
            }
        )
        if freq != 0:
            base_freqs.append((k, freq))

    # ---------- 1~12 次谐波按精确频率分组：只保留跨关系重合 ----------
    buckets: dict[Fraction, list[tuple[int, int]]] = {}
    for rel_no, f in base_freqs:
        for h in range(1, MAX_HARMONIC + 1):
            buckets.setdefault(f * h, []).append((rel_no, h))
    groups = []
    for freq in sorted(buckets):
        members = sorted(buckets[freq])  # (关系录入序号, 谐波次数) 升序
        if len({r for r, _ in members}) < 2:
            continue  # 至少两条不同关系才算重合
        groups.append(
            {
                "frequency": _fraction_json(freq),
                "members": [
                    {"relation": r, "harmonic": h} for r, h in members
                ],
            }
        )

    return {
        "status": "ok",
        "shafts": shaft_ids,
        "rank": len(pivot_cols),
        "free_count": len(free_cols),
        "free_shafts": [shaft_ids[c] for c in free_cols],
        "harmonic_max": MAX_HARMONIC,
        "relations": relations_out,
        "groups": groups,
    }


def _brake_set_valid(
    rel_rows: list[list[Fraction]],
    n: int,
    brake_cols: list[int],
    target_col: int,
    protected_col: int,
) -> bool:
    """判定一个候选制动集合是否有效（全程有理数）。

    在关系齐次方程 Aω=0 上为每根制动轴追加单位约束行 ω_j=0 后做 RREF。
      · 目标轴在所有允许运动中恒为零 ⟺ 目标轴是主元列且其主元行对所有
        自由列的系数均为 0（单位行 e_target 已落在行空间中）；
      · 保护轴仍存在转速非零的允许运动 ⟺ 保护轴是自由列，或其主元行存在
        非零自由列系数（单位行 e_protected 不在行空间中）。
    追加的都是齐次行，方程组永不会矛盾。
    """
    mat = [row[:] for row in rel_rows]
    for j in brake_cols:
        unit = [Fraction(0)] * (n + 1)
        unit[j] = Fraction(1)
        mat.append(unit)
    pivot_cols = _rref(mat, n)
    pivot_row_of = {c: i for i, c in enumerate(pivot_cols)}
    free_cols = [j for j in range(n) if j not in pivot_row_of]

    # 目标轴：主元行必须是只在目标列取 1 的单位行（ω_target = 0 被强制）
    rt = pivot_row_of.get(target_col)
    if rt is None or any(mat[rt][c] != 0 for c in free_cols):
        return False

    # 保护轴：自由列（可取任意转速），或主元行含非零自由列系数（随其非零）
    if protected_col in pivot_row_of:
        rp = pivot_row_of[protected_col]
        if all(mat[rp][c] == 0 for c in free_cols):
            return False
    return True


def brake_plans(req: BrakePlansRequest) -> dict:
    """最少制动轴方案：枚举可制动轴全部子集，求制动轴数最少的有效方案。

    轴与传动关系的校验和 /api/solve 完全复用（已知转速、转矩均不参与，故
    不校验）。候选集合有效性的判定见 _brake_set_valid：仅当目标轴在所有
    允许运动中转速恒为零、且存在保护轴转速非零的允许运动时有效。枚举可
    制动轴的全部子集（含空集），保留有效集合中制动轴数最少的那些；方案内
    轴按录入顺序排列，方案间按该顺序的轴列表字典序排列。目标轴与保护轴
    相同、可制动轴重复或包含未知/目标/保护轴时返回 HTTP 400。
    """
    prepared = _prepare(req, with_known=False)
    shaft_ids = prepared["shaft_ids"]
    index = prepared["index"]
    n = prepared["n"]
    rel_rows = prepared["rows"]  # 仅传动关系齐次行：已知转速不参与

    errors: list[str] = []

    # ---------- 目标轴 / 保护轴 ----------
    target_sid = _parse_int(req.target)
    if target_sid is None:
        errors.append("目标轴编号非法")
    elif target_sid not in index:
        errors.append(f"目标轴 {target_sid} 未在轴列表中定义")

    protected_sid = _parse_int(req.protected)
    if protected_sid is None:
        errors.append("保护轴编号非法")
    elif protected_sid not in index:
        errors.append(f"保护轴 {protected_sid} 未在轴列表中定义")

    if (
        target_sid is not None and protected_sid is not None
        and target_sid in index and protected_sid in index
        and target_sid == protected_sid
    ):
        errors.append(f"目标轴与保护轴不能相同（均为轴 {target_sid}）")

    # ---------- 可制动轴：可空（此时只有空集一个候选） ----------
    if req.brakeable is None:
        errors.append("缺少可制动轴列表 brakeable（不勾选任何轴时应传空数组）")
        brakeable_sids: list[int] = []
    else:
        brakeable_sids = []
        seen_brake: set[int] = set()
        for k, raw in enumerate(req.brakeable):
            sid = _parse_int(raw)
            if sid is None:
                errors.append(f"可制动轴 {k + 1}：轴编号非法")
                continue
            if sid not in index:
                errors.append(f"可制动轴 {k + 1}：轴 {sid} 未在轴列表中定义")
                continue
            if sid in seen_brake:
                errors.append(f"可制动轴 {k + 1}：轴 {sid} 重复勾选")
                continue
            if target_sid is not None and sid == target_sid:
                errors.append(f"可制动轴 {k + 1}：轴 {sid} 是目标轴，不能同时作为可制动轴")
            if protected_sid is not None and sid == protected_sid:
                errors.append(f"可制动轴 {k + 1}：轴 {sid} 是保护轴，不能同时作为可制动轴")
            seen_brake.add(sid)
            brakeable_sids.append(sid)

    if errors:
        raise HTTPException(status_code=400, detail={"errors": errors})

    assert target_sid is not None and protected_sid is not None
    target_col = index[target_sid]
    protected_col = index[protected_sid]
    # 可制动轴按轴录入顺序排列：子集枚举、方案内排序与字典序均以此为准
    brakeable_sids.sort(key=lambda s: index[s])
    brakeable_cols = [index[s] for s in brakeable_sids]

    # ---------- 枚举全部子集（共 2^k，k ≤ 轴数−2 ≤ 10），按规模从小到大 ----------
    candidates = 1 << len(brakeable_cols)
    best: list[tuple[int, ...]] = []
    min_size: int | None = None
    for size in range(len(brakeable_cols) + 1):
        found = [
            tuple(cols)
            for cols in combinations(brakeable_cols, size)
            if _brake_set_valid(rel_rows, n, list(cols), target_col, protected_col)
        ]
        if found:  # 按规模升序：首个出现有效集合的规模即最少制动轴数
            best = found
            min_size = size
            break

    base = {
        "shafts": shaft_ids,
        "target": target_sid,
        "protected": protected_sid,
        "brakeable": brakeable_sids,
        "candidate_count": candidates,
    }

    if min_size is None:
        return {
            **base,
            "status": "none",
            "min_brakes": None,
            "count": 0,
            "plans": [],
            "message": (
                f"在全部 {candidates} 个候选制动集合（含空集）中，没有任何制动方案能"
                f"使目标轴 {target_sid} 在所有允许运动中恒不转动、同时保护轴 {protected_sid} "
                "仍可转动。请增加可制动轴或调整传动关系后重试。"
            ),
        }

    best.sort()  # 按轴录入位置的轴列表字典序
    return {
        **base,
        "status": "ok",
        "min_brakes": min_size,
        "count": len(best),
        "plans": [[shaft_ids[c] for c in cols] for cols in best],
        "message": (
            f"候选集合共 {candidates} 个（含空集），按制动轴数由少到多评估："
            f"最少需要制动 {min_size} 根轴，有 {len(best)} 个并列最优方案；每个方案下目标轴 "
            f"{target_sid} 在所有允许运动中转速恒为零，且存在保护轴 {protected_sid} 转速非零的允许运动。"
        ),
    }
