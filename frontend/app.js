'use strict';

/* ============================== 状态 ============================== */

const LIMITS = { shafts: [2, 12], relations: [1, 24], known: [1, 4], tknown: [1, 12], tunknown: [1, 12] };

const state = {
  shafts: [1, 2, 3, 4],
  relations: [
    { type: 'planetary', s: '1', r: '2', c: '3', zs: '20', zr: '60' },
    { type: 'external', a: '4', b: '2', za: '10', zb: '10' },
  ],
  known: [
    { shaft: '2', speed: '0' },    // 齿圈固定
    { shaft: '1', speed: '100' },  // 太阳轮 100 rpm
  ],
  // 目标转速配齿：关系序号（1 基）、齿数字段、目标轴、目标转速
  options: { relation: 1, field: 'zr', shaft: 3, speed: '25' },
  // 理想传动转矩平衡：已知外加转矩（N·m）与待求转矩轴；其余轴转矩视为 0
  torque: {
    known: [{ shaft: '1', torque: '10' }],
    unknown: ['2', '3'],
  },
  // 传动链路追踪：起点/终点轴编号（字符串，与轴列表的录入值对应）
  path: { start: '1', end: '3' },
  // 最少制动轴方案：目标轴、保护轴（互异）与勾选的可制动轴（不含前两者）
  brake: { target: '4', protected: '3', brakeable: ['1', '2'] },
};

// 各关系类型允许替换的齿数字段
const RELATION_FIELDS = {
  external: ['za', 'zb'],
  internal: ['za', 'zb'],
  planetary: ['zs', 'zr'],
};
const FIELD_LABEL = { za: 'za', zb: 'zb', zs: 'zs', zr: 'zr' };

const $ = (sel) => document.querySelector(sel);
const esc = (v) =>
  String(v).replace(/[&<>"']/g, (ch) =>
    ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[ch]));

const parseInt10 = (v) => {
  if (typeof v === 'number') return Number.isInteger(v) ? v : null;
  if (typeof v !== 'string') return null;
  const s = v.trim();
  return /^[+-]?\d+$/.test(s) ? parseInt(s, 10) : null;
};

const parseTeeth = (v) => {
  const n = parseInt10(v);
  return n !== null && n >= 1 && n <= 500 ? n : null;
};

// 合法转速：有符号整数或 p/q 分数字符串
function parseSpeed(v) {
  if (typeof v !== 'string') return false;
  const s = v.trim();
  if (!s) return false;
  if (/^[+-]?\d+$/.test(s)) return true;
  const m = s.match(/^([+-]?\d+)\s*\/\s*([+-]?\d+)$/);
  return !!m && parseInt(m[2], 10) !== 0;
}

// 外加转矩与转速同为有符号整数或分数字符串，共用同一校验
const parseTorque = parseSpeed;

/* ============================== 渲染：录入区 ============================== */

function shaftOptions(selected, includeDangling) {
  let html = state.shafts.map((id) =>
    `<option value="${id}" ${String(id) === String(selected) ? 'selected' : ''}>${id}</option>`
  ).join('');
  const sel = parseInt10(selected);
  if (includeDangling && (sel === null || !state.shafts.includes(sel))) {
    html = `<option value="${esc(selected)}" selected>轴 ${esc(selected)}（未定义）</option>` + html;
  }
  return html;
}

function omega(ref) {
  const label = ref === '' || ref === null || ref === undefined ? '?' : esc(ref);
  return `ω<sub>${label}</sub>`;
}

function relationEquation(r) {
  const t = (v) => (v === undefined || v === null ? '' : String(v).trim());
  if (r.type === 'external' || r.type === 'internal') {
    const za = t(r.za);
    const zb = t(r.zb);
    const op = r.type === 'external' ? '+' : '−';
    return `${za || '?'}·${omega(r.a)} ${op} ${zb || '?'}·${omega(r.b)} = 0`;
  }
  const zs = t(r.zs);
  const zr = t(r.zr);
  return `${zs || '?'}(${omega(r.s)}−${omega(r.c)}) + ${zr || '?'}(${omega(r.r)}−${omega(r.c)}) = 0`;
}

function relationFields(r, i) {
  const sel = (value, includeDangling) => shaftOptions(value, includeDangling);
  if (r.type === 'planetary') {
    return `
      <div class="grid">
        <div class="field"><label>太阳轮轴 s</label>
          <select data-section="rel" data-index="${i}" data-name="s">${sel(r.s, true)}</select></div>
        <div class="field"><label>齿圈轴 r</label>
          <select data-section="rel" data-index="${i}" data-name="r">${sel(r.r, true)}</select></div>
        <div class="field"><label>行星架轴 c</label>
          <select data-section="rel" data-index="${i}" data-name="c">${sel(r.c, true)}</select></div>
        <div class="field"><label>齿数 zs（1~500）</label>
          <input type="text" inputmode="numeric" value="${esc(r.zs)}" class="${parseTeeth(r.zs) === null ? 'invalid' : ''}"
                 data-section="rel" data-index="${i}" data-name="zs" /></div>
        <div class="field"><label>齿数 zr（1~500）</label>
          <input type="text" inputmode="numeric" value="${esc(r.zr)}" class="${parseTeeth(r.zr) === null ? 'invalid' : ''}"
                 data-section="rel" data-index="${i}" data-name="zr" /></div>
      </div>
      <div class="equation">${relationEquation(r)}</div>`;
  }
  const label = r.type === 'external' ? '外啮合' : '内啮合';
  const op = r.type === 'external' ? '+' : '−';
  return `
    <div class="grid">
      <div class="field"><label>轴 a</label>
        <select data-section="rel" data-index="${i}" data-name="a">${sel(r.a, true)}</select></div>
      <div class="field"><label>轴 b</label>
        <select data-section="rel" data-index="${i}" data-name="b">${sel(r.b, true)}</select></div>
      <div class="field"><label>齿数 za（1~500）</label>
        <input type="text" inputmode="numeric" value="${esc(r.za)}" class="${parseTeeth(r.za) === null ? 'invalid' : ''}"
               data-section="rel" data-index="${i}" data-name="za" /></div>
      <div class="field"><label>齿数 zb（1~500）</label>
        <input type="text" inputmode="numeric" value="${esc(r.zb)}" class="${parseTeeth(r.zb) === null ? 'invalid' : ''}"
               data-section="rel" data-index="${i}" data-name="zb" /></div>
    </div>
    <div class="equation dim dim-eq">${label}：za·ωa ${op} zb·ωb = 0</div>
    <div class="equation">${relationEquation(r)}</div>`;
}

const TYPE_LABEL = { external: '外啮合 za·ωa + zb·ωb = 0', internal: '内啮合 za·ωa − zb·ωb = 0', planetary: '行星组 zs(ωs−ωc) + zr(ωr−ωc) = 0' };

function renderShafts() {
  const dupCount = {};
  state.shafts.forEach((raw) => {
    const id = parseInt10(raw);
    if (id !== null) dupCount[id] = (dupCount[id] || 0) + 1;
  });
  $('#shaft-list').innerHTML = state.shafts.map((raw, i) => {
    const id = parseInt10(raw);
    const bad = id === null || dupCount[id] > 1;
    return `
    <div class="row-item">
      <div class="row-head">
        <span class="row-index">#${i + 1}</span>
        <div class="field" style="min-width:180px">
          <label>轴编号（唯一整数）</label>
          <input type="number" step="1" value="${esc(raw)}" class="${bad ? 'invalid' : ''}"
                 data-section="shaft" data-index="${i}" data-name="id" />
        </div>
        <button class="btn btn-del" data-act="del-shaft" data-index="${i}"
                ${state.shafts.length <= LIMITS.shafts[0] ? 'disabled' : ''}>删除</button>
      </div>
    </div>`;
  }).join('');
  $('#shaft-count').textContent = `${state.shafts.length} / ${LIMITS.shafts[1]}`;
  $('#btn-add-shaft').disabled = state.shafts.length >= LIMITS.shafts[1];
}

function renderRelations() {
  $('#relation-list').innerHTML = state.relations.map((r, i) => `
    <div class="row-item">
      <div class="row-head">
        <span class="row-index">关系 ${i + 1}</span>
        <div class="field" style="min-width:300px">
          <label>类型</label>
          <select data-section="rel" data-index="${i}" data-name="type">
            <option value="external" ${r.type === 'external' ? 'selected' : ''}>${TYPE_LABEL.external}</option>
            <option value="internal" ${r.type === 'internal' ? 'selected' : ''}>${TYPE_LABEL.internal}</option>
            <option value="planetary" ${r.type === 'planetary' ? 'selected' : ''}>${TYPE_LABEL.planetary}</option>
          </select>
        </div>
        <button class="btn btn-del" data-act="del-rel" data-index="${i}"
                ${state.relations.length <= LIMITS.relations[0] ? 'disabled' : ''}>删除</button>
      </div>
      ${relationFields(r, i)}
    </div>`).join('');
  $('#relation-count').textContent = `${state.relations.length} / ${LIMITS.relations[1]}`;
  $('#btn-add-relation').disabled = state.relations.length >= LIMITS.relations[1];
}

function renderKnown() {
  $('#known-list').innerHTML = state.known.map((k, i) => `
    <div class="row-item">
      <div class="row-head">
        <span class="row-index">已知 ${i + 1}</span>
        <div class="field" style="min-width:160px">
          <label>轴</label>
          <select data-section="known" data-index="${i}" data-name="shaft">${shaftOptions(k.shaft, true)}</select>
        </div>
        <div class="field" style="min-width:200px">
          <label>转速 ω（rpm，整数或分数，如 120、-3/2）</label>
          <input type="text" value="${esc(k.speed)}" class="${parseSpeed(k.speed) ? '' : 'invalid'}"
                 data-section="known" data-index="${i}" data-name="speed" />
        </div>
        <button class="btn btn-del" data-act="del-known" data-index="${i}"
                ${state.known.length <= LIMITS.known[0] ? 'disabled' : ''}>删除</button>
      </div>
    </div>`).join('');
  $('#known-count').textContent = `${state.known.length} / ${LIMITS.known[1]}`;
  $('#btn-add-known').disabled = state.known.length >= LIMITS.known[1];
}

// 目标转速配齿表单：关系、齿数字段（随关系类型变化）、目标轴、目标转速
function renderOptions() {
  const opt = state.options;
  // 关系序号越界（关系被删除）时收回首条；字段不适用该关系类型时取该类型首个字段
  if (!Number.isInteger(opt.relation) || opt.relation < 1 || opt.relation > state.relations.length) {
    opt.relation = 1;
  }
  const rel = state.relations[opt.relation - 1];
  const allowed = RELATION_FIELDS[rel.type] || ['za', 'zb'];
  if (!allowed.includes(opt.field)) opt.field = allowed[0];

  $('#opt-relation').innerHTML = state.relations.map((r, i) =>
    `<option value="${i + 1}" ${opt.relation === i + 1 ? 'selected' : ''}>关系 ${i + 1} · ${TYPE_SHORT[r.type]}</option>`
  ).join('');
  $('#opt-field').innerHTML = allowed.map((f) =>
    `<option value="${f}" ${opt.field === f ? 'selected' : ''}>${FIELD_LABEL[f]}</option>`
  ).join('');
  $('#opt-shaft').innerHTML = shaftOptions(opt.shaft, true);
  const speedEl = $('#opt-speed');
  speedEl.value = opt.speed ?? '';
  speedEl.classList.toggle('invalid', !parseSpeed(opt.speed));
}

// 转矩平衡：已知/待求两侧的轴冲突（同一轴重复出现或同时已知与待求）
function torqueShaftIssues() {
  const ids = (arr) => arr.map(parseInt10).filter((x) => x !== null);
  const knownIds = ids(state.torque.known.map((k) => k.shaft));
  const unknownIds = ids(state.torque.unknown);
  const dupOf = (arr) => {
    const seen = new Set();
    const dup = new Set();
    arr.forEach((x) => { if (seen.has(x)) dup.add(x); seen.add(x); });
    return dup;
  };
  const knownDup = dupOf(knownIds);
  const unknownDup = dupOf(unknownIds);
  const unknownSet = new Set(unknownIds);
  const knownSet = new Set(knownIds);
  return {
    knownBad: new Set(knownIds.filter((x) => knownDup.has(x) || unknownSet.has(x))),
    unknownBad: new Set(unknownIds.filter((x) => unknownDup.has(x) || knownSet.has(x))),
  };
}

function renderTorque() {
  const issues = torqueShaftIssues();
  $('#torque-known-list').innerHTML = state.torque.known.map((k, i) => {
    const sid = parseInt10(k.shaft);
    const badShaft = sid !== null && issues.knownBad.has(sid);
    return `
    <div class="row-item">
      <div class="row-head">
        <span class="row-index">转矩 ${i + 1}</span>
        <div class="field" style="min-width:140px">
          <label>轴</label>
          <select class="${badShaft ? 'invalid' : ''}"
                  data-section="tknown" data-index="${i}" data-name="shaft">${shaftOptions(k.shaft, true)}</select>
        </div>
        <div class="field" style="min-width:200px">
          <label>外加转矩 τ（N·m，整数或分数，如 10、-3/2）</label>
          <input type="text" value="${esc(k.torque)}" class="${parseTorque(k.torque) ? '' : 'invalid'}"
                 data-section="tknown" data-index="${i}" data-name="torque" />
        </div>
        <button class="btn btn-del" data-act="del-torque-known" data-index="${i}"
                ${state.torque.known.length <= LIMITS.tknown[0] ? 'disabled' : ''}>删除</button>
      </div>
    </div>`;
  }).join('');
  $('#torque-known-count').textContent = `${state.torque.known.length} / ${LIMITS.tknown[1]}`;
  $('#btn-add-torque-known').disabled = state.torque.known.length >= LIMITS.tknown[1];

  $('#torque-unknown-list').innerHTML = state.torque.unknown.map((raw, i) => {
    const sid = parseInt10(raw);
    const badShaft = sid !== null && issues.unknownBad.has(sid);
    return `
    <div class="row-item">
      <div class="row-head">
        <span class="row-index">待求 ${i + 1}</span>
        <div class="field" style="min-width:140px">
          <label>轴</label>
          <select class="${badShaft ? 'invalid' : ''}"
                  data-section="tunknown" data-index="${i}" data-name="shaft">${shaftOptions(raw, true)}</select>
        </div>
        <button class="btn btn-del" data-act="del-torque-unknown" data-index="${i}"
                ${state.torque.unknown.length <= LIMITS.tunknown[0] ? 'disabled' : ''}>删除</button>
      </div>
    </div>`;
  }).join('');
  $('#torque-unknown-count').textContent = `${state.torque.unknown.length} / ${LIMITS.tunknown[1]}`;
  $('#btn-add-torque-unknown').disabled = state.torque.unknown.length >= LIMITS.tunknown[1];
}

function renderAll() {
  renderShafts();
  renderRelations();
  renderKnown();
  renderOptions();
  renderTorque();
  renderPath();
  renderBrake();
}

// 传动链路追踪表单：起点/终点轴；轴被删除导致选择悬空时收回首/末根轴
function renderPath() {
  const p = state.path;
  const ids = state.shafts.map(String);
  if (!ids.includes(String(p.start))) p.start = String(state.shafts[0] ?? '');
  if (!ids.includes(String(p.end))) p.end = String(state.shafts[state.shafts.length - 1] ?? '');
  $('#path-start').innerHTML = shaftOptions(p.start, false);
  $('#path-end').innerHTML = shaftOptions(p.end, false);
}

// 最少制动轴方案表单：目标轴/保护轴下拉，其余轴作为可制动轴复选项。
// 目标与保护相同在下拉上标红；目标/保护变化时把它们从可制动勾选集合中剔除。
function renderBrake() {
  const b = state.brake;
  const ids = state.shafts.map(String);
  if (!ids.includes(String(b.target))) b.target = String(state.shafts[0] ?? '');
  if (!ids.includes(String(b.protected))) {
    b.protected = String(state.shafts[state.shafts.length - 1] ?? '');
  }
  b.brakeable = b.brakeable.filter((s) => ids.includes(String(s)));

  $('#brake-target').innerHTML = shaftOptions(b.target, false);
  $('#brake-protected').innerHTML = shaftOptions(b.protected, false);
  $('#brake-target').classList.toggle('invalid', String(b.target) === String(b.protected));
  $('#brake-protected').classList.toggle('invalid', String(b.target) === String(b.protected));

  const reserved = new Set([String(b.target), String(b.protected)]);
  const checks = state.shafts.map((id) => {
    const sid = String(id);
    const locked = reserved.has(sid);
    const checked = !locked && b.brakeable.includes(sid);
    return `
      <label class="brake-check${locked ? ' locked' : ''}">
        <input type="checkbox" data-brake-shaft="${esc(sid)}" ${checked ? 'checked' : ''}
               ${locked ? 'disabled' : ''} />
        <span class="brake-shaft">轴 ${esc(sid)}</span>
        ${locked ? `<span class="badge">${sid === String(b.target) ? '目标轴' : '保护轴'}</span>` : ''}
      </label>`;
  }).join('');
  $('#brake-check-list').innerHTML = checks;
  $('#brake-count').textContent = `${b.brakeable.length} / ${Math.max(state.shafts.length - 2, 0)}`;
}

/* ============================== 输入交互 ============================== */

function clearResult() {
  const box = $('#result');
  box.hidden = true;
  box.innerHTML = '';
  clearConflictHighlights();
}

// 配齿结果与普通求解结果互为独立区域；任何录入变化都一并清空，避免展示过期答案
function clearOptionsResult() {
  const box = $('#options-result');
  box.hidden = true;
  box.innerHTML = '';
}

// 转矩平衡结果同为独立区域，随任何录入变化清空
function clearTorqueResult() {
  const box = $('#torque-result');
  box.hidden = true;
  box.innerHTML = '';
}

// 链路追踪结果同为独立区域，随任何录入变化清空（含所选链路/必经关系高亮）
function clearPathResult() {
  const box = $('#path-result');
  box.hidden = true;
  box.innerHTML = '';
  clearPathHighlights();
}

// 谐波诊断结果同为独立区域，随任何录入变化清空（含参与重合关系高亮）
function clearHarmonicsResult() {
  const box = $('#harmonics-result');
  box.hidden = true;
  box.innerHTML = '';
  clearHarmonicHighlights();
}

// 制动方案结果同为独立区域，随任何录入变化清空
function clearBrakeResult() {
  const box = $('#brake-result');
  box.hidden = true;
  box.innerHTML = '';
}

function clearAllResults() {
  clearResult();
  clearOptionsResult();
  clearTorqueResult();
  clearPathResult();
  clearHarmonicsResult();
  clearBrakeResult();
}

function clearConflictHighlights() {
  document.querySelectorAll('.row-item.conflict').forEach((el) => el.classList.remove('conflict'));
}

// 链路追踪：清除关系录入行上的所选链路/必经关系高亮
function clearPathHighlights() {
  document.querySelectorAll('.row-item.path-step, .row-item.path-essential')
    .forEach((el) => el.classList.remove('path-step', 'path-essential'));
}

// 链路追踪：所选链路经过的关系行蓝色高亮，所有最短链路必经的关系行绿色高亮
function highlightPath(path, essential) {
  clearPathHighlights();
  const list = $('#relation-list');
  if (!list) return;
  const essentialSet = new Set(essential || []);
  (path || []).forEach((s) => {
    const item = list.children[s.relation - 1];
    if (item) item.classList.add(essentialSet.has(s.relation) ? 'path-essential' : 'path-step');
  });
}

// 谐波诊断：清除关系录入行上的重合参与高亮
function clearHarmonicHighlights() {
  document.querySelectorAll('.row-item.harmonic')
    .forEach((el) => el.classList.remove('harmonic'));
}

// 谐波诊断：参与任一重合组的关系行紫色高亮
function highlightHarmonics(groups) {
  clearHarmonicHighlights();
  const list = $('#relation-list');
  if (!list) return;
  const involved = new Set();
  (groups || []).forEach((g) => (g.members || []).forEach((m) => involved.add(m.relation)));
  involved.forEach((k) => {
    const item = list.children[k - 1];
    if (item) item.classList.add('harmonic');
  });
}

// 无解冲突定位：给对应关系/已知转速输入行加高亮（序号取响应中的原始 1 基序号）
function highlightConflict(conflict) {
  clearConflictHighlights();
  (conflict || []).forEach((c) => {
    const list = c.type === 'relation' ? '#relation-list' : c.type === 'known' ? '#known-list' : null;
    const item = list && $(list) ? $(list).children[c.index - 1] : null;
    if (item) item.classList.add('conflict');
  });
}

function nextShaftId() {
  const nums = state.shafts.map(parseInt10).filter((n) => n !== null);
  let id = (nums.reduce((m, x) => Math.max(m, x), 0) + 1);
  const used = new Set(nums);
  while (used.has(id)) id++;
  return id;
}

function emptyRelation() {
  if (state.relations.some((r) => r.type !== 'planetary')) {
    return { type: 'external', a: String(state.shafts[0] ?? ''), b: String(state.shafts[1] ?? ''), za: '20', zb: '40' };
  }
  return {
    type: 'planetary',
    s: String(state.shafts[0] ?? ''),
    r: String(state.shafts[1] ?? ''),
    c: String(state.shafts[2] ?? state.shafts[0] ?? ''),
    zs: '20', zr: '60',
  };
}

document.addEventListener('click', (e) => {
  const btn = e.target.closest('[data-act]');
  if (!btn) return;
  clearAllResults();
  const act = btn.dataset.act;
  const i = btn.dataset.index !== undefined ? parseInt(btn.dataset.index, 10) : null;

  if (act === 'add-shaft') {
    state.shafts.push(nextShaftId());
    renderAll();
  } else if (act === 'del-shaft') {
    state.shafts.splice(i, 1);
    renderAll();
  } else if (act === 'add-relation') {
    state.relations.push(emptyRelation());
    renderAll();
  } else if (act === 'del-rel') {
    state.relations.splice(i, 1);
    renderAll();
  } else if (act === 'add-known') {
    state.known.push({ shaft: String(state.shafts[0] ?? ''), speed: '0' });
    renderAll();
  } else if (act === 'del-known') {
    state.known.splice(i, 1);
    renderAll();
  } else if (act === 'add-torque-known') {
    state.torque.known.push({ shaft: String(state.shafts[0] ?? ''), torque: '0' });
    renderTorque();
  } else if (act === 'del-torque-known') {
    state.torque.known.splice(i, 1);
    renderTorque();
  } else if (act === 'add-torque-unknown') {
    // 默认选第一根既未待求也未录入已知转矩的轴
    const used = new Set([
      ...state.torque.unknown.map(String),
      ...state.torque.known.map((k) => String(k.shaft)),
    ]);
    const pick = state.shafts.map(String).find((s) => !used.has(s)) ?? String(state.shafts[0] ?? '');
    state.torque.unknown.push(pick);
    renderTorque();
  } else if (act === 'del-torque-unknown') {
    state.torque.unknown.splice(i, 1);
    renderTorque();
  } else if (act === 'solve') {
    void runSolve();
  } else if (act === 'gear-options') {
    void runGearOptions();
  } else if (act === 'torque-balance') {
    void runTorqueBalance();
  } else if (act === 'trace-path') {
    void runPathTrace();
  } else if (act === 'mesh-harmonics') {
    void runMeshHarmonics();
  } else if (act === 'brake-plans') {
    void runBrakePlans();
  } else if (act === 'apply-option') {
    applyGearOption(btn.dataset.relation, btn.dataset.field, btn.dataset.teeth);
  }
});

// 轴编号：失焦（change）时提交，避免键入过程中打断焦点
document.addEventListener('change', (e) => {
  const el = e.target.closest('[data-edit-target], [data-section]');
  if (!el) return;
  const { section, index, name } = el.dataset;
  if (section !== 'shaft') return; // 其余 change 走统一处理
  clearAllResults();
  const n = parseInt10(el.value);
  state.shafts[index] = n === null ? el.value : n;
  renderAll();
});

// 关系/已知转速：change（select）与 input（文本）
document.addEventListener('input', onFieldChange);
document.addEventListener('change', (e) => {
  const el = e.target.closest('[data-section]');
  if (el && el.dataset.section !== 'shaft') onFieldChange(e);
});

function onFieldChange(e) {
  const el = e.target.closest('[data-section]');
  if (!el) return;
  const { section, index, name } = el.dataset;
  if (section === 'shaft') return;
  clearAllResults();

  if (section === 'rel') {
    if (name === 'type') {
      const old = state.relations[index];
      // 尽量沿用已选轴与齿数，缺失字段补默认值，避免 undefined
      if (el.value === 'planetary') {
        state.relations[index] = {
          type: 'planetary',
          s: old.s ?? old.a ?? String(state.shafts[0] ?? ''),
          r: old.r ?? old.b ?? String(state.shafts[1] ?? ''),
          c: old.c ?? String(state.shafts[2] ?? state.shafts[0] ?? ''),
          zs: old.zs ?? old.za ?? '20',
          zr: old.zr ?? old.zb ?? '60',
        };
      } else {
        state.relations[index] = {
          type: el.value,
          a: old.a ?? old.s ?? String(state.shafts[0] ?? ''),
          b: old.b ?? old.r ?? String(state.shafts[1] ?? ''),
          za: old.za ?? old.zs ?? '20',
          zb: old.zb ?? old.zr ?? '40',
        };
      }
      renderRelations();
      renderOptions(); // 关系类型变化后，配齿可选齿数字段随之收敛
      return;
    }
    state.relations[index][name] = el.value;
    if (name === 'za' || name === 'zb' || name === 'zs' || name === 'zr') {
      el.classList.toggle('invalid', parseTeeth(el.value) === null);
    }
    updateRowEquation(parseInt(index, 10));
  } else if (section === 'known') {
    state.known[index][name] = el.value;
    if (name === 'speed') el.classList.toggle('invalid', !parseSpeed(el.value));
  } else if (section === 'tknown') {
    state.torque.known[index][name] = el.value;
    if (name === 'torque') el.classList.toggle('invalid', !parseTorque(el.value));
    if (name === 'shaft') renderTorque(); // 轴变化后重算已知/待求冲突高亮
  } else if (section === 'tunknown') {
    state.torque.unknown[index] = el.value;
    renderTorque(); // 轴变化后重算重复/冲突高亮
  }
}

function updateRowEquation(i) {
  const item = $('#relation-list').children[i];
  if (!item) return;
  const eq = item.querySelector('.equation:not(.dim-eq)');
  if (eq) eq.innerHTML = relationEquation(state.relations[i]);
}

/* ============================== 目标转速配齿 ============================== */

// 配齿表单的任何改动都清空旧结果；关系变化时按其类型收敛可选字段
$('#opt-relation').addEventListener('change', (e) => {
  clearAllResults();
  state.options.relation = parseInt(e.target.value, 10);
  renderOptions();
});
$('#opt-field').addEventListener('change', (e) => {
  clearAllResults();
  state.options.field = e.target.value;
});
$('#opt-shaft').addEventListener('change', (e) => {
  clearAllResults();
  state.options.shaft = e.target.value;
});
$('#opt-speed').addEventListener('input', (e) => {
  clearAllResults();
  state.options.speed = e.target.value;
  e.target.classList.toggle('invalid', !parseSpeed(e.target.value));
});

// 链路追踪表单：起终点改动清空旧结果
$('#path-start').addEventListener('change', (e) => {
  clearAllResults();
  state.path.start = e.target.value;
});
$('#path-end').addEventListener('change', (e) => {
  clearAllResults();
  state.path.end = e.target.value;
});

// 最少制动轴方案表单：目标/保护轴改动后从可制动集合中剔除并清空旧结果
$('#brake-target').addEventListener('change', (e) => {
  clearAllResults();
  state.brake.target = e.target.value;
  state.brake.brakeable = state.brake.brakeable
    .filter((s) => String(s) !== String(e.target.value) && String(s) !== String(state.brake.protected));
  renderBrake();
});
$('#brake-protected').addEventListener('change', (e) => {
  clearAllResults();
  state.brake.protected = e.target.value;
  state.brake.brakeable = state.brake.brakeable
    .filter((s) => String(s) !== String(e.target.value) && String(s) !== String(state.brake.target));
  renderBrake();
});
// 可制动轴复选框（事件委托，重渲染后仍然生效）
document.addEventListener('change', (e) => {
  const cb = e.target.closest('input[data-brake-shaft]');
  if (!cb) return;
  clearAllResults();
  const sid = cb.dataset.brakeShaft;
  if (cb.checked) {
    if (!state.brake.brakeable.map(String).includes(sid)) state.brake.brakeable.push(sid);
  } else {
    state.brake.brakeable = state.brake.brakeable.filter((s) => String(s) !== sid);
  }
  $('#brake-count').textContent =
    `${state.brake.brakeable.length} / ${Math.max(state.shafts.length - 2, 0)}`;
});

async function runGearOptions() {
  const btn = $('#btn-options');
  const box = $('#options-result');
  btn.disabled = true;
  btn.textContent = '枚举中…';
  clearOptionsResult();
  box.hidden = false;
  try {
    const payload = buildPayload();
    payload.target = {
      relation: state.options.relation,
      field: state.options.field,
      shaft: parseInt10(state.options.shaft),
      speed: String(state.options.speed ?? '').trim(),
    };
    const resp = await fetch('/api/gear-options', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const data = await resp.json().catch(() => null);
    if (!resp.ok) {
      lastOptionsData = null;
      renderOptionsErrors(extractErrors(resp.status, data));
    } else {
      lastOptionsData = data;
      renderGearOptions(data);
    }
  } catch (err) {
    lastOptionsData = null;
    renderOptionsErrors([`网络或服务异常：${err.message}`]);
  } finally {
    btn.disabled = false;
    btn.textContent = '配 齿';
    box.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
  }
}

// 最近一次配齿结果：回填齿数后用于重新渲染清单（录入变化时清空作废）
let lastOptionsData = null;

// 点击某个匹配齿数：回填到原关系的对应齿数字段，清除旧求解结果，不动其他输入。
// 本函数由文档级 click 委托触发，委托处理已先调用 clearAllResults()。
function applyGearOption(relationStr, field, teeth) {
  const relIdx = parseInt(relationStr, 10) - 1;
  const rel = state.relations[relIdx];
  if (!rel || !teeth) return;
  rel[field] = String(teeth);
  renderRelations();                 // 更新齿数字段与方程展示
  const box = $('#options-result');
  box.hidden = false;                // 普通求解结果已被委托清除；配齿清单重新渲染保留
  if (lastOptionsData) renderGearOptions(lastOptionsData);
  renderApplyNote(relIdx + 1, field, teeth);
  const item = $('#relation-list').children[relIdx];
  if (item) {
    const input = item.querySelector(`input[data-name="${field}"]`);
    if (input) {
      input.focus();
      input.select();
    }
    item.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
  }
}

function renderApplyNote(relationNo, field, teeth) {
  const note = $('#options-apply-note');
  if (note) {
    note.innerHTML = `✓ 已将 <code>关系 ${relationNo}</code> 的齿数 <code>${esc(field)}</code> 回填为 <strong>${esc(teeth)}</strong>，旧求解结果已清除；可再次“求解”查看该齿数下的各轴转速，其他输入未改动。`;
    note.classList.add('applied');
  }
}

/* ============================== 求解 ============================== */

function buildPayload() {
  return {
    shafts: state.shafts.map((id) => ({ id: parseInt10(id) })),
    relations: state.relations.map((r) => {
      if (r.type === 'planetary') {
        return {
          type: r.type,
          s: parseInt10(r.s), r: parseInt10(r.r), c: parseInt10(r.c),
          zs: String(r.zs ?? '').trim(), zr: String(r.zr ?? '').trim(),
        };
      }
      return {
        type: r.type,
        a: parseInt10(r.a), b: parseInt10(r.b),
        za: String(r.za ?? '').trim(), zb: String(r.zb ?? '').trim(),
      };
    }),
    known: state.known.map((k) => ({ shaft: parseInt10(k.shaft), speed: String(k.speed ?? '').trim() })),
  };
}

async function runSolve() {
  const btn = $('#btn-solve');
  const box = $('#result');
  btn.disabled = true;
  btn.textContent = '求解中…';
  clearResult();
  try {
    const payload = buildPayload();
    const resp = await fetch('/api/solve', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const data = await resp.json().catch(() => null);
    if (!resp.ok) {
      renderErrors(extractErrors(resp.status, data));
    } else {
      renderResult(data, payload);
    }
  } catch (err) {
    renderErrors([`网络或服务异常：${err.message}`]);
  } finally {
    btn.disabled = false;
    btn.textContent = '求 解';
    box.hidden = false;
    box.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
  }
}

function extractErrors(status, data) {
  if (data && data.detail && Array.isArray(data.detail.errors)) return data.detail.errors;
  if (status === 422 && data && Array.isArray(data.detail)) {
    return data.detail.map((d) => `${d.loc.join('.')}：${d.msg}`);
  }
  if (data && typeof data.detail === 'string') return [data.detail];
  return [`请求失败（HTTP ${status}），请检查输入。`];
}

/* ============================== 渲染：结果 ============================== */

const MINUS = '−'; // U+2212

// 后端分数的 num/den 为十进制整数字符串（大整数若当 JSON 数字解析会被
// 双精度舍入），这里一律按字符串处理，绝不经过 Number 运算。
function fracParts(f) {
  let num = String(f && f.num != null ? f.num : '0').trim();
  let den = String(f && f.den != null ? f.den : '1').trim();
  if (!/^[+-]?\d+$/.test(num)) num = '0';
  if (!/^[+-]?\d+$/.test(den) || /^[+-]?0+$/.test(den)) den = '1';
  let neg = num.startsWith('-');
  if (den.startsWith('-')) neg = !neg; // 规范分数分母恒正，此处仅作兜底
  return {
    neg,
    absNum: num.replace(/^[+-]/, ''),
    absDen: den.replace(/^[+-]/, ''),
    isOne: den.replace(/^[+-]/, '') === '1',
  };
}

// 堆叠分数：分母为 1 时显示为整数；负号整体前置；零显示为 0
function fracHTML(f) {
  const p = fracParts(f);
  if (p.isOne) {
    return `<span class="frac whole">${p.neg ? MINUS : ''}${p.absNum}</span>`;
  }
  return `${p.neg ? `<span class="neg">${MINUS}</span>` : ''}
    <span class="frac"><span class="f-num">${p.absNum}</span><span class="f-den">${p.absDen}</span></span>`;
}

// 单个系数（分数）与自由变量 t 拼成带号项；系数为 ±1 时省略“1·”
function termParts(coef, freeIndex) {
  const p = fracParts(coef);
  let body;
  if (p.isOne && p.absNum === '1') body = `t<sub>${freeIndex}</sub>`;
  else if (p.isOne) body = `${p.absNum}·t<sub>${freeIndex}</sub>`;
  else body = `<span class="frac"><span class="f-num">${p.absNum}</span><span class="f-den">${p.absDen}</span></span>·t<sub>${freeIndex}</sub>`;
  return { sign: p.neg ? MINUS : '+', body };
}

function bodyOfConst(c) {
  const p = fracParts(c);
  if (p.isOne) return p.absNum;
  return `<span class="frac"><span class="f-num">${p.absNum}</span><span class="f-den">${p.absDen}</span></span>`;
}

function exprHTML(expr, sym = 'ω', unit = 'rpm') {
  const lhs = `${sym}<sub>${esc(expr.shaft)}</sub>`;
  if (expr.free) {
    return `
      <div class="expr">
        <span class="lhs">${lhs}</span><span>=</span>
        <span class="rhs">t<sub>${expr.free_index}</sub></span>
        <span class="badge free">自由变量</span>
      </div>`;
  }
  // 主元轴：ω = 常数项 + Σ 系数·t（首项带正负号，其余项带 + / −）
  const constP = fracParts(expr.constant);
  let rhs;
  if (constP.absNum === '0' && expr.terms.length) {
    // 常数项为 0：直接以首个带号项开头，避免“0 + t₁”这类多余的零
    const [first, ...rest] = expr.terms.map((t) => termParts(t.coef, t.free_index));
    rhs = [
      first.sign === MINUS ? `${MINUS} ${first.body}` : first.body,
      ...rest.map((p) => `${p.sign} ${p.body}`),
    ].join(' ');
  } else {
    const parts = [{ sign: constP.neg ? MINUS : '', body: bodyOfConst(expr.constant) }];
    expr.terms.forEach((t) => parts.push(termParts(t.coef, t.free_index)));
    rhs = parts
      .map((p, idx) => (idx === 0 ? (p.sign ? `${p.sign} ${p.body}` : p.body) : `${p.sign} ${p.body}`))
      .join(' ');
  }
  return `
    <div class="expr">
      <span class="lhs">${lhs}</span><span>=</span>
      <span class="rhs">${rhs}</span>
      <span class="unit">${unit}</span>
    </div>`;
}

const TYPE_SHORT = { external: '外啮合', internal: '内啮合', planetary: '行星组' };

// 描述单条冲突条件：关系给出方程原文，已知转速给出轴与转速；数据取自本次请求快照
function conflictItemHTML(c, payload) {
  if (c.type === 'relation') {
    const r = payload && payload.relations[c.index - 1];
    const label = r && TYPE_SHORT[r.type] ? TYPE_SHORT[r.type] : '传动关系';
    const eq = r ? relationEquation(r) : '';
    return `<li><code>关系 ${c.index}</code><span class="conflict-tag">${label}</span>
      <span class="conflict-eq">${eq}</span></li>`;
  }
  if (c.type === 'known') {
    const k = payload && payload.known[c.index - 1];
    const shaft = k ? k.shaft : '?';
    const speed = k ? k.speed : '?';
    return `<li><code>已知转速 ${c.index}</code><span class="conflict-tag">轴 ${esc(shaft)}</span>
      <span class="conflict-eq">ω<sub>${esc(shaft)}</sub> = ${esc(speed)} rpm</span></li>`;
  }
  return `<li><code>条件 ${c.index}</code></li>`;
}

function renderResult(data, payload) {
  const box = $('#result');
  if (data.status === 'unique') {
    const rows = data.values.map((v) => `
      <tr>
        <td><code>轴 ${esc(v.shaft)}</code></td>
        <td class="num">ω<sub>${esc(v.shaft)}</sub></td>
        <td class="num">${fracHTML(v.fraction)}</td>
        <td class="num">rpm</td>
      </tr>`).join('');
    box.innerHTML = `
      <div class="banner unique">✓ 唯一解
        <span class="meta">秩 = ${data.rank}，未知轴数 = ${data.shafts.length}，方程组定解。</span>
      </div>
      <table class="speed-table">
        <thead><tr><th>轴</th><th>变量</th><th>转速（最简有符号分数）</th><th>单位</th></tr></thead>
        <tbody>${rows}</tbody>
      </table>`;
  } else if (data.status === 'underdetermined') {
    const freeDecl = data.expressions
      .filter((e) => e.free)
      .map((e) => `<code>t<sub>${e.free_index}</sub> = ω<sub>${esc(e.shaft)}</sub></code>`)
      .join('，');
    const exprs = data.expressions.map((e) => exprHTML(e)).join('');
    box.innerHTML = `
      <div class="banner underdetermined">△ 欠定（有无穷多解）
        <span class="meta">秩 = ${data.rank}，未知轴数 = ${data.shafts.length}，
        自由轴 ${data.free_count} 根：${data.free_shafts.map(esc).join('、')}</span>
      </div>
      <div class="free-note">消元顺序按轴录入顺序；非主元轴取为自由变量：${freeDecl}（t 可取任意有理数，单位 rpm）</div>
      <div class="expr-list">${exprs}</div>`;
  } else if (data.status === 'inconsistent') {
    const conflict = Array.isArray(data.conflict) ? data.conflict : [];
    const items = conflict.map((c) => conflictItemHTML(c, payload)).join('');
    box.innerHTML = `
      <div class="banner inconsistent">✕ 无解
        <span class="meta">已定位 ${conflict.length} 条相互冲突的条件：该集合本身无解，删去其中任意一条即恢复有解。</span>
      </div>
      <p>${esc(data.message || '方程组矛盾，该齿轮系无解。')}</p>
      <div class="conflict-box">
        <h3>冲突条件（按录入顺序，不可再缩减）</h3>
        <ul class="conflict-list">${items}</ul>
        <p class="conflict-note">对应输入行已用红色高亮标出；修改或删除其中任一条件后重新求解即可。</p>
      </div>`;
    highlightConflict(conflict);
  } else {
    box.innerHTML = `<div class="error-box"><h3>未知结果状态</h3></div>`;
  }
}

function renderErrors(errors) {
  const box = $('#result');
  box.innerHTML = `
    <div class="error-box">
      <h3>输入有误，共 ${errors.length} 项：</h3>
      <ul>${errors.map((m) => `<li>${esc(m)}</li>`).join('')}</ul>
    </div>`;
}

/* ============================== 渲染：配齿结果 ============================== */

function renderOptionsErrors(errors) {
  const box = $('#options-result');
  box.innerHTML = `
    <div class="error-box">
      <h3>配齿输入有误，共 ${errors.length} 项：</h3>
      <ul>${errors.map((m) => `<li>${esc(m)}</li>`).join('')}</ul>
    </div>`;
}

// 配齿命中清单：按 |齿数-原齿数|、齿数升序（后端已排好）；每行可点选回填
function renderGearOptions(data) {
  const box = $('#options-result');
  const matches = Array.isArray(data.matches) ? data.matches : [];
  const head = `
    <div class="banner options">⚙ 目标转速配齿
      <span class="meta">关系 ${data.relation} 的齿数 ${esc(data.field)} 枚举 1～500；
      目标轴 ${esc(data.target_shaft)} 精确等于 ${fracHTML(data.target_speed)} rpm
      （原齿数 ${esc(data.original_teeth)}）</span>
    </div>
    <p id="options-apply-note" class="apply-note">点击下表中的齿数即可回填到原关系，并清除旧求解结果；其他输入保持不变。</p>`;

  if (!matches.length) {
    box.innerHTML = `${head}
      <div class="error-box options-empty">
        <h3>无匹配齿数</h3>
        <p>在 1～500 范围内，没有任何齿数能使目标轴 ${esc(data.target_shaft)} 的转速
        精确等于 ${fracHTML(data.target_speed)} rpm（矛盾候选已舍弃；欠定但目标轴转速依赖自由变量的也不计入）。</p>
      </div>`;
    return;
  }

  const rows = matches.map((m) => `
    <tr>
      <td class="num"><button class="btn btn-apply" data-act="apply-option"
            data-relation="${data.relation}" data-field="${esc(data.field)}"
            data-teeth="${esc(m.teeth)}">${esc(m.teeth)}</button></td>
      <td class="num">${esc(m.distance)}</td>
      <td class="num">${fracHTML(m.speed)}</td>
      <td class="num">rpm</td>
    </tr>`).join('');
  box.innerHTML = `${head}
    <div class="options-meta">共 <strong>${matches.length}</strong> 个匹配齿数（按与原齿数差的绝对值、齿数升序排列）</div>
    <table class="speed-table options-table">
      <thead><tr><th>${esc(data.field)}（点击回填）</th><th>与原齿数之差</th><th>目标轴 ${esc(data.target_shaft)} 转速</th><th>单位</th></tr></thead>
      <tbody>${rows}</tbody>
    </table>`;
}

/* ============================== 理想传动转矩平衡 ============================== */

async function runTorqueBalance() {
  const btn = $('#btn-torque');
  const box = $('#torque-result');
  btn.disabled = true;
  btn.textContent = '求解中…';
  clearTorqueResult();
  box.hidden = false;
  try {
    const base = buildPayload(); // 轴与传动关系同转速求解；已知转速不参与，不上送
    const payload = {
      shafts: base.shafts,
      relations: base.relations,
      known_torques: state.torque.known.map((k) => ({
        shaft: parseInt10(k.shaft),
        torque: String(k.torque ?? '').trim(),
      })),
      unknown_shafts: state.torque.unknown.map((s) => parseInt10(s)),
    };
    const resp = await fetch('/api/torque-balance', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const data = await resp.json().catch(() => null);
    if (!resp.ok) {
      renderTorqueErrors(extractErrors(resp.status, data));
    } else {
      renderTorqueResult(data);
    }
  } catch (err) {
    renderTorqueErrors([`网络或服务异常：${err.message}`]);
  } finally {
    btn.disabled = false;
    btn.textContent = '平衡求解';
    box.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
  }
}

function renderTorqueErrors(errors) {
  const box = $('#torque-result');
  box.innerHTML = `
    <div class="error-box">
      <h3>转矩平衡输入有误，共 ${errors.length} 项：</h3>
      <ul>${errors.map((m) => `<li>${esc(m)}</li>`).join('')}</ul>
    </div>`;
}

// 已知/视为 0 的上下文说明：三种状态共用
function torqueContextNote(data) {
  const knownLine = (data.known_torques || [])
    .map((t) => `τ<sub>${esc(t.shaft)}</sub> = ${fracHTML(t.torque)}`)
    .join('，');
  const zeroLine = (data.zero_shafts || []).map((s) => `τ<sub>${esc(s)}</sub>`).join('、');
  return `
    <div class="free-note">已知外加转矩：${knownLine || '无'}（N·m）；
    其余轴外加转矩视为 0${zeroLine ? `（${zeroLine}）` : ''}；已知转速不参与本计算。
    独立平衡方程数（A 的零空间维数）= ${data.nullity}。</div>`;
}

function renderTorqueResult(data) {
  const box = $('#torque-result');
  if (data.status === 'unique') {
    const knownMap = new Map((data.known_torques || []).map((t) => [String(t.shaft), t.torque]));
    const solvedMap = new Map((data.values || []).map((v) => [String(v.shaft), v.fraction]));
    const rows = data.shafts.map((sid) => {
      const key = String(sid);
      let frac;
      let badge;
      if (knownMap.has(key)) {
        frac = knownMap.get(key);
        badge = '<span class="badge known">已知</span>';
      } else if (solvedMap.has(key)) {
        frac = solvedMap.get(key);
        badge = '<span class="badge solved">待求</span>';
      } else {
        frac = { num: '0', den: '1' };
        badge = '<span class="badge">视为 0</span>';
      }
      return `
      <tr>
        <td><code>轴 ${esc(sid)}</code></td>
        <td class="num">τ<sub>${esc(sid)}</sub></td>
        <td class="num">${fracHTML(frac)}</td>
        <td class="num">N·m</td>
        <td>${badge}</td>
      </tr>`;
    }).join('');
    box.innerHTML = `
      <div class="banner unique">✓ 唯一平衡解
        <span class="meta">平衡方程 = ${data.nullity}，秩 = ${data.rank}，待求轴 ${data.unknown_shafts.length} 根，转矩唯一定解。</span>
      </div>
      ${torqueContextNote(data)}
      <table class="speed-table">
        <thead><tr><th>轴</th><th>变量</th><th>外加转矩（最简有符号分数）</th><th>单位</th><th>来源</th></tr></thead>
        <tbody>${rows}</tbody>
      </table>`;
  } else if (data.status === 'underdetermined') {
    const freeDecl = data.expressions
      .filter((e) => e.free)
      .map((e) => `<code>t<sub>${e.free_index}</sub> = τ<sub>${esc(e.shaft)}</sub></code>`)
      .join('，');
    const exprs = data.expressions.map((e) => exprHTML(e, 'τ', 'N·m')).join('');
    box.innerHTML = `
      <div class="banner underdetermined">△ 欠定（有无穷多平衡解）
        <span class="meta">平衡方程 = ${data.nullity}，秩 = ${data.rank}，待求轴 ${data.unknown_shafts.length} 根，
        自由变量 ${data.free_count} 个：轴 ${data.free_shafts.map(esc).join('、')}</span>
      </div>
      ${torqueContextNote(data)}
      <div class="free-note">消元顺序按轴录入顺序；非主元待求轴取为自由变量：${freeDecl}（t 可取任意有理数，单位 N·m）</div>
      <div class="expr-list">${exprs}</div>`;
  } else if (data.status === 'inconsistent') {
    box.innerHTML = `
      <div class="banner inconsistent">✕ 无平衡解
        <span class="meta">平衡方程 = ${data.nullity}，外加转矩不满足理想传动的功率平衡。</span>
      </div>
      ${torqueContextNote(data)}
      <p>${esc(data.message || '平衡方程组矛盾，该齿轮系无转矩平衡解。')}</p>
      <p class="conflict-note">理想（无损、无惯性）传动要求：对关系系数矩阵 A 零空间中的任意允许运动 v，
      外加转矩均满足 τ·v = 0。请调整已知外加转矩的取值，或重新划分已知/待求轴。</p>`;
  } else {
    box.innerHTML = `<div class="error-box"><h3>未知结果状态</h3></div>`;
  }
}

/* ============================== 传动链路追踪 ============================== */

async function runPathTrace() {
  const btn = $('#btn-path');
  const box = $('#path-result');
  btn.disabled = true;
  btn.textContent = '追踪中…';
  clearPathResult();
  box.hidden = false;
  try {
    const base = buildPayload(); // 轴与传动关系同转速求解；已知转速、转矩均不参与
    const payload = {
      shafts: base.shafts,
      relations: base.relations,
      start: parseInt10(state.path.start),
      end: parseInt10(state.path.end),
    };
    const resp = await fetch('/api/transmission-paths', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const data = await resp.json().catch(() => null);
    if (!resp.ok) {
      renderPathErrors(extractErrors(resp.status, data));
    } else {
      renderPathResult(data);
    }
  } catch (err) {
    renderPathErrors([`网络或服务异常：${err.message}`]);
  } finally {
    btn.disabled = false;
    btn.textContent = '追踪链路';
    box.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
  }
}

function renderPathErrors(errors) {
  const box = $('#path-result');
  box.innerHTML = `
    <div class="error-box">
      <h3>链路追踪输入有误，共 ${errors.length} 项：</h3>
      <ul>${errors.map((m) => `<li>${esc(m)}</li>`).join('')}</ul>
    </div>`;
}

// 单步链路端点：轴 7（太阳轮 s，齿数 20）；行星架无齿数则省略
function pathEndHTML(e) {
  const teeth = e.teeth === null || e.teeth === undefined ? '' : `，齿数 ${esc(e.teeth)}`;
  return `轴 ${esc(e.shaft)}（${esc(e.role_label)}${teeth}）`;
}

function renderPathResult(data) {
  const box = $('#path-result');
  if (data.status === 'disconnected') {
    box.innerHTML = `
      <div class="banner inconsistent">✕ 不连通
        <span class="meta">最短链路总数 = 0</span>
      </div>
      <p>${esc(data.message || '起点与终点之间不存在传动链路。')}</p>`;
    return;
  }
  if (data.length === 0) {
    box.innerHTML = `
      <div class="banner unique">✓ 同轴空链路
        <span class="meta">最短链路总数 = 1，长度 = 0</span>
      </div>
      <p>${esc(data.message || '起点与终点为同一根轴：一条长度为零的空链路，不经过任何传动关系。')}</p>`;
    return;
  }
  const essential = Array.isArray(data.essential) ? data.essential : [];
  const essentialSet = new Set(essential);
  const steps = (data.path || []).map((s) => {
    const teethSummary = Object.entries(s.teeth || {})
      .map(([k, v]) => `${esc(k)} = ${esc(v)}`).join('，');
    const isEssential = essentialSet.has(s.relation);
    return `
      <li class="path-step-item${isEssential ? ' essential' : ''}">
        <span class="step-no">第 ${s.step} 步</span>
        <code>关系 ${esc(s.relation)}</code>
        <span class="path-tag">${esc(TYPE_SHORT[s.type] || s.type)}</span>
        <span class="path-step-text">${pathEndHTML(s.from)} → ${pathEndHTML(s.to)}</span>
        <span class="path-teeth">${teethSummary}</span>
        ${isEssential ? '<span class="badge solved">必经</span>' : ''}
      </li>`;
  }).join('');
  const essentialText = essential.length
    ? essential.map((k) => `<code>关系 ${esc(k)}</code>`).join('、')
    : '无（每条关系都至少被一条最短链路绕过）';
  box.innerHTML = `
    <div class="banner unique">✓ 最短链路长度 = ${esc(data.length)}
      <span class="meta">最短链路总数 = ${esc(data.count)}（并行关系分别计数）；
      下示按每步（关系录入序号、到达轴录入序号）字典序最小的链路。</span>
    </div>
    <ol class="path-list">${steps}</ol>
    <div class="free-note">全部 ${esc(data.count)} 条最短链路必经的关系：${essentialText}。
    关系录入区中，所选链路经过的关系以蓝色高亮，必经关系以绿色高亮。</div>`;
  highlightPath(data.path || [], essential);
}

/* ============================== 啮合谐波重合诊断 ============================== */

async function runMeshHarmonics() {
  const btn = $('#btn-harmonics');
  const box = $('#harmonics-result');
  btn.disabled = true;
  btn.textContent = '诊断中…';
  clearHarmonicsResult();
  box.hidden = false;
  try {
    const payload = buildPayload(); // 轴/传动关系/已知转速与“求解”完全相同
    const resp = await fetch('/api/mesh-harmonics', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const data = await resp.json().catch(() => null);
    if (!resp.ok) {
      renderHarmonicsErrors(extractErrors(resp.status, data));
    } else {
      renderHarmonicsResult(data, payload);
    }
  } catch (err) {
    renderHarmonicsErrors([`网络或服务异常：${err.message}`]);
  } finally {
    btn.disabled = false;
    btn.textContent = '谐波诊断';
    box.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
  }
}

function renderHarmonicsErrors(errors) {
  const box = $('#harmonics-result');
  box.innerHTML = `
    <div class="error-box">
      <h3>谐波诊断输入有误，共 ${errors.length} 项：</h3>
      <ul>${errors.map((m) => `<li>${esc(m)}</li>`).join('')}</ul>
    </div>`;
}

// 单条关系的基频公式：外/内啮合 f = za·|ωa|/60；行星组 f = zs·|ωs−ωc|/60
function harmonicFormulaHTML(r, k) {
  const t = (v) => (v === undefined || v === null || String(v).trim() === '' ? '?' : esc(String(v).trim()));
  if (r && (r.type === 'external' || r.type === 'internal')) {
    return `f<sub>${k}</sub> = ${t(r.za)}·|${omega(r.a)}| / 60`;
  }
  if (r && r.type === 'planetary') {
    return `f<sub>${k}</sub> = ${t(r.zs)}·|${omega(r.s)} − ${omega(r.c)}| / 60`;
  }
  return `f<sub>${k}</sub>`;
}

function renderHarmonicsResult(data, payload) {
  const box = $('#harmonics-result');
  if (data.status === 'inconsistent') {
    // 模型无解：与“求解”相同的冲突定位展示（红色高亮对应输入行）
    const conflict = Array.isArray(data.conflict) ? data.conflict : [];
    const items = conflict.map((c) => conflictItemHTML(c, payload)).join('');
    box.innerHTML = `
      <div class="banner inconsistent">✕ 无解
        <span class="meta">转速方程组矛盾，无法计算啮合基频；已定位 ${conflict.length} 条相互冲突的条件：该集合本身无解，删去其中任意一条即恢复有解。</span>
      </div>
      <p>${esc(data.message || '方程组矛盾，该齿轮系无解。')}</p>
      <div class="conflict-box">
        <h3>冲突条件（按录入顺序，不可再缩减）</h3>
        <ul class="conflict-list">${items}</ul>
        <p class="conflict-note">对应输入行已用红色高亮标出；修改或删除其中任一条件后重新诊断即可。</p>
      </div>`;
    highlightConflict(conflict);
    return;
  }

  const rels = Array.isArray(data.relations) ? data.relations : [];
  const groups = Array.isArray(data.groups) ? data.groups : [];
  const hmax = data.harmonic_max || 12;
  const undetermined = rels.filter((r) => !r.determined).map((r) => r.relation);
  const zeroRels = rels
    .filter((r) => r.determined && r.frequency && fracParts(r.frequency).absNum === '0')
    .map((r) => r.relation);

  const rows = rels.map((r) => {
    const rel = payload && payload.relations[r.relation - 1];
    const freqCell = r.determined
      ? fracHTML(r.frequency)
      : '<span class="badge free">未确定</span>';
    return `
      <tr>
        <td><code>关系 ${esc(r.relation)}</code></td>
        <td>${esc(TYPE_SHORT[r.type] || r.type)}</td>
        <td class="num">${harmonicFormulaHTML(rel, r.relation)}</td>
        <td class="num">${freqCell}</td>
        <td class="num">${r.determined ? 'Hz' : ''}</td>
      </tr>`;
  }).join('');

  const notes = [];
  if (undetermined.length) {
    notes.push(`欠定：${undetermined.map((k) => `关系 ${k}`).join('、')} 的基频仍依赖自由变量，标为未确定，不参与重合比较。`);
  }
  if (zeroRels.length) {
    notes.push(`${zeroRels.map((k) => `关系 ${k}`).join('、')} 的基频为 0，不产生谐波，不参与重合比较。`);
  }
  const noteHTML = notes.length ? `<div class="free-note">${notes.map(esc).join('<br />')}</div>` : '';

  let groupsHTML;
  if (groups.length) {
    const items = groups.map((g) => {
      const members = (g.members || [])
        .map((m) => `<code>关系 ${esc(m.relation)}</code> ${esc(m.harmonic)} 次`)
        .join('＝');
      return `
        <li class="harmonic-group-item">
          <span class="harmonic-freq">${fracHTML(g.frequency)} Hz</span>
          <span class="harmonic-members">${members}</span>
        </li>`;
    }).join('');
    groupsHTML = `
      <div class="options-meta">共 <strong>${groups.length}</strong> 组谐波重合（仅列出包含至少两条不同关系的重合组；组内按关系录入序号、谐波次数排序）</div>
      <ol class="harmonic-list">${items}</ol>
      <div class="free-note">参与重合的关系已在录入区以紫色高亮。</div>`;
  } else {
    groupsHTML = `
      <div class="error-box options-empty">
        <h3>无谐波重合</h3>
        <p>在 1～${hmax} 次谐波范围内，没有任何精确频率被至少两条不同关系共享。</p>
      </div>`;
  }

  const meta = data.free_count > 0
    ? `欠定：自由轴 ${data.free_count} 根（${(data.free_shafts || []).map(esc).join('、')}）；未确定关系 ${undetermined.length} 条`
    : `定解：全部 ${rels.length} 条关系基频确定`;
  box.innerHTML = `
    <div class="banner options">♪ 啮合谐波重合诊断
      <span class="meta">谐波次数 1～${hmax}；${meta}</span>
    </div>
    ${noteHTML}
    <table class="speed-table harmonics-table">
      <thead><tr><th>关系</th><th>类型</th><th>基频公式</th><th>基频</th><th>单位</th></tr></thead>
      <tbody>${rows}</tbody>
    </table>
    ${groupsHTML}`;
  highlightHarmonics(groups);
}

/* ============================== 最少制动轴方案 ============================== */

async function runBrakePlans() {
  const btn = $('#btn-brake');
  const box = $('#brake-result');
  btn.disabled = true;
  btn.textContent = '枚举中…';
  clearBrakeResult();
  box.hidden = false;
  try {
    const base = buildPayload(); // 只使用轴与传动关系；已知转速、转矩均不参与
    const payload = {
      shafts: base.shafts,
      relations: base.relations,
      target: parseInt10(state.brake.target),
      protected: parseInt10(state.brake.protected),
      brakeable: state.brake.brakeable.map((s) => parseInt10(s)),
    };
    const resp = await fetch('/api/brake-plans', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const data = await resp.json().catch(() => null);
    if (!resp.ok) {
      renderBrakeErrors(extractErrors(resp.status, data));
    } else {
      renderBrakeResult(data);
    }
  } catch (err) {
    renderBrakeErrors([`网络或服务异常：${err.message}`]);
  } finally {
    btn.disabled = false;
    btn.textContent = '求制动方案';
    box.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
  }
}

function renderBrakeErrors(errors) {
  const box = $('#brake-result');
  box.innerHTML = `
    <div class="error-box">
      <h3>制动方案输入有误，共 ${errors.length} 项：</h3>
      <ul>${errors.map((m) => `<li>${esc(m)}</li>`).join('')}</ul>
    </div>`;
}

function renderBrakeResult(data) {
  const box = $('#brake-result');
  const plans = Array.isArray(data.plans) ? data.plans : [];
  const head = `
    <div class="banner ${data.status === 'ok' ? 'unique' : 'underdetermined'}">
      ${data.status === 'ok'
        ? `✓ 最少制动轴数 = ${esc(data.min_brakes)}`
        : '△ 无可行制动方案'}
      <span class="meta">目标轴 ${esc(data.target)} 须恒不转动，保护轴 ${esc(data.protected)} 必须保持可转；
      共枚举 ${esc(data.candidate_count)} 个候选制动集合（含空集）。</span>
    </div>
    <div class="free-note">可制动轴：${
      (data.brakeable || []).length
        ? (data.brakeable || []).map((s) => `轴 ${esc(s)}`).join('、')
        : '无（未勾选任何轴，仅评估空集）'
    }。判定基于关系齐次方程 Aω = 0 的允许运动空间，全程有理数精确计算，不使用已知转速或转矩。</div>`;

  if (data.status !== 'ok') {
    box.innerHTML = `${head}
      <div class="error-box options-empty">
        <h3>无制动方案</h3>
        <p>${esc(data.message || '没有任何候选制动集合满足要求。')}</p>
      </div>`;
    return;
  }

  const items = plans.map((plan, i) => {
    const shafts = plan.length
      ? plan.map((s) => `<span class="brake-chip">轴 ${esc(s)}</span>`).join('')
      : '<span class="brake-chip empty">空集（不制动任何轴）</span>';
    return `
      <li class="brake-plan-item">
        <span class="step-no">方案 ${i + 1}</span>
        <span class="brake-plan-shafts">${shafts}</span>
        <span class="badge solved">${plan.length} 根制动轴</span>
      </li>`;
  }).join('');
  box.innerHTML = `${head}
    <div class="options-meta">共 <strong>${plans.length}</strong> 个并列最优方案
    （方案内按轴录入顺序，方案间按该顺序的轴列表字典序排列）</div>
    <ol class="brake-plan-list">${items}</ol>
    <div class="free-note">${esc(data.message || '')}</div>`;
}

/* ============================== 启动 ============================== */

renderAll();
