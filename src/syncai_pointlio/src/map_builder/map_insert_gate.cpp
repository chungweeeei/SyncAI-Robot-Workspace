#include "syncai_pointlio/map_builder/map_insert_gate.h"

MapInsertGate::MapInsertGate(const Config & config)
: m_enabled(config.map_gate_enabled),
  m_max_speed(config.map_gate_max_speed),
  m_min_effective_ratio(config.map_gate_min_effective_ratio),
  m_min_near(config.map_gate_min_near),
  m_close_after(config.map_gate_close_after),
  m_open_after(config.map_gate_open_after)
{
}

MapInsertGate::Verdict MapInsertGate::judge(double speed, int near, int effective)
{
  Verdict v;
  v.speed = speed;
  v.near = near;
  v.effective = effective;
  if (!m_enabled) return v;

  // Strictly over / under: a scan exactly at a threshold passes.
  v.speed_failed = speed > m_max_speed;
  if (near >= m_min_near && near > 0) {
    v.ratio = static_cast<double>(effective) / near;
    v.ratio_failed = v.ratio < m_min_effective_ratio;
  }
  const bool failed = v.speed_failed || v.ratio_failed;

  if (m_open) {
    m_streak = failed ? m_streak + 1 : 0;
    if (m_streak >= m_close_after) {
      m_open = false;
      m_streak = 0;
      v.closed_now = true;
    }
    v.insert = !failed;
  } else {
    m_streak = failed ? 0 : m_streak + 1;
    if (m_streak >= m_open_after) {
      m_open = true;
      m_streak = 0;
      v.opened_now = true;
    }
    // The scan that reopens the gate is the first one inserted again.
    v.insert = m_open;
  }
  return v;
}
