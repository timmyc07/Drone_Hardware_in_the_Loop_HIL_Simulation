import { validateFollowSettings } from './follow.js';

const reasons = {
  pilot: '操作員停止', 'mode-change': '飛行模式變更', 'settings-changed': '設定變更',
  'follower-disarmed': '跟隨機已上鎖', 'follower-crashed': '跟隨機碰撞', 'follower-failsafe': '跟隨機進入保護模式',
  'safety-action': '安全保護動作', 'leader-missing': '領航機不存在', 'leader-stale': '領航機資料逾時',
  'leader-invalid': '領航機資料無效', 'leader-disarmed': '領航機已上鎖', 'leader-crashed': '領航機碰撞',
  'leader-failsafe': '領航機進入保護模式', 'leader-link-loss': '領航機連線中斷', 'leader-landed': '領航機已降落',
  'target-range': '超出最大距離', 'target-geofence': '目標超出圍欄', 'target-terrain': '目標地形無效',
  'target-ground': '目標低於地面', 'target-invalid': '目標無效',
};
export function formatFollow(settings, state, follower, vehicles) {
  const id = state?.leaderId || settings.leaderId;
  const leader = vehicles.find(v => v.id === id);
  const name = id ? `${leader?.name || id}（${id}）` : '未選擇';
  let text = `領航機：${name} · ${state?.active ? '跟飛中' : state ? `已停止：${reasons[state.reason] || state.reason || '—'}` : '尚未開始'}`;
  if (leader?.snapshot?.position && follower?.position)
    text += ` · 距離 ${Math.hypot(...leader.snapshot.position.map((v, i) => v - follower.position[i])).toFixed(1)} m`;
  if (state?.targetPosition) {
    const [n, e, d] = state.targetPosition;
    text += ` · 目標 北 ${n.toFixed(1)}、東 ${e.toFixed(1)}、高 ${(-d).toFixed(1)} m`;
  }
  return text;
}

export function createFollowPanel({ apply }) {
  const $ = id => document.getElementById(`follow-${id}`);
  const ids = ['leader', 'frame', 'x', 'y', 'altitude', 'distance', 'lost', 'yaw'];
  const drafts = new Map();
  let id, readonly = false, busy = false, current;
  const changed = e => JSON.stringify(e.draft) !== JSON.stringify(e.base);
  const raw = s => ({ leader: s.leaderId || '', frame: s.frame, x: String(s.offset.x), y: String(s.offset.y), altitude: String(s.offset.altitude), distance: String(s.maxDistanceM), lost: s.lostAction, yaw: s.yawMode });
  function render() {
    if (!current) return;
    const leader = $('leader'), value = current.draft.leader;
    leader.replaceChildren(new Option('不選擇', ''), ...current.vehicles.filter(v => v.id !== id).map(v => new Option(`${v.name}（${v.id}）`, v.id)));
    if (value && !current.vehicles.some(v => v.id === value && v.id !== id)) leader.add(new Option(`不存在：${value}`, value));
    for (const key of ids) { $(key).value = current.draft[key]; $(key).disabled = readonly || busy; }
    $('apply').disabled = readonly || busy;
    $('status').textContent = readonly ? '重播設定為唯讀。' : changed(current) ? '有未套用的跟飛草稿。' : '已與目前設定一致。';
  }
  for (const key of ids) $(key).oninput = () => {
    if (readonly || busy || !current) return;
    current.draft[key] = $(key).value;
    $('status').textContent = changed(current) ? '有未套用的跟飛草稿。' : '已與目前設定一致。';
  };
  $('apply').onclick = async () => {
    if (readonly || busy || !current) return;
    const entry = current, submittedId = id, d = entry.draft;
    const number = s => s.trim() === '' ? NaN : Number(s);
    try {
      const settings = validateFollowSettings({ leaderId: d.leader || null, frame: d.frame, offset: { x: number(d.x), y: number(d.y), altitude: number(d.altitude) }, maxDistanceM: number(d.distance), lostAction: d.lost, yawMode: d.yaw });
      busy = true; render();
      await apply(submittedId, settings);
      if (drafts.get(submittedId) === entry) {
        entry.base = raw(settings); entry.draft = structuredClone(entry.base);
      }
      if (current === entry) { render(); $('status').textContent = '跟飛設定已套用。'; }
    } catch (error) { if (current === entry) $('status').textContent = `無法套用：${error.message}`; }
    finally { busy = false; if (current === entry) { const message = $('status').textContent; render(); $('status').textContent = message; } }
  };
  return {
    open(vehicleId, settings, vehicles, isReadonly = false) {
      id = vehicleId; readonly = isReadonly;
      const base = raw(settings), old = drafts.get(id);
      if (readonly) current = { base, draft: structuredClone(base), vehicles };
      else {
        current = old || { base, draft: structuredClone(base), vehicles };
        const dirty = changed(current);
        current.base = base; current.vehicles = vehicles;
        if (!dirty) current.draft = structuredClone(base);
        drafts.set(id, current);
      }
      render();
    },
    retain(vehicleIds) { const ids = new Set(vehicleIds); for (const key of drafts.keys()) if (!ids.has(key)) drafts.delete(key); },
    clear() { drafts.clear(); current = null; },
  };
}
