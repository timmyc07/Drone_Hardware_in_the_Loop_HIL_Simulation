export function formatCollisionPrevention(state) {
  if (!state) return "幾何碰撞預防：關閉";
  const status = !state.applicable ? "已啟用，等待位置控制模式" : state.reason === "insufficient-clearance" ? "餘距不足" : state.active ? "減速中" : "正常";
  const obstacle = state.obstacleType ? ` · ${state.obstacleType === "peer" ? "載具" : state.obstacleType === "nist" ? "NIST 障礙" : "障礙盒"} ${state.obstacleId}` : "";
  return `幾何碰撞預防：${status}${obstacle} · 設定安全餘量 ${state.clearanceM.toFixed(1)} m`;
}
