export function readPilotInput(keys, gamepad) {
  const axis = (i) => {
    const value = gamepad?.axes?.[i];
    return Number.isFinite(value) && Math.abs(value) > 0.08
      ? Math.max(-1, Math.min(1, value))
      : 0;
  };
  const keyboard = (negative, positive, fallback) => {
    const n = keys.has(negative),
      p = keys.has(positive);
    return n || p ? Number(p) - Number(n) : fallback;
  };
  return {
    roll: keyboard("KeyA", "KeyD", axis(0)),
    pitch: keyboard("KeyW", "KeyS", axis(1)),
    yaw: keyboard("ArrowLeft", "ArrowRight", axis(2)),
    climb: keyboard("ArrowDown", "ArrowUp", -axis(3)),
  };
}

export function takeoffPlayAction(snapshot) {
  if (!snapshot || ["crashed", "failsafe"].includes(snapshot.status))
    return "play";
  if (
    ["runup", "roll", "rotate", "climb"].includes(snapshot.runway.takeoffPhase)
  )
    return "resume";
  const moving = Math.hypot(...snapshot.velocity) >= 0.25;
  if (
    snapshot.runway.weightOnWheels &&
    !moving &&
    !(snapshot.navigation.mode === "land" && snapshot.armed)
  )
    return "launch";
  return "play";
}
