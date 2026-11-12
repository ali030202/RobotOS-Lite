export type Mode = 'STARTING' | 'IDLE' | 'RUNNING' | 'FAULT' | 'ESTOP' | 'OFFLINE'

export interface RobotStatus {
  mode: Mode
  fault_code: string
  sensor_fresh: boolean
  position: number[]
  target: number[]
  active_task_id: string
  progress: number
  observed_at: string
  age_ms: number | null
}

export interface Task {
  task_id: string
  state: 'RUNNING' | 'CANCEL_REQUESTED' | 'SUCCEEDED' | 'CANCELED' | 'ABORTED'
  code: string | null
  progress: number
  position: number[]
}
