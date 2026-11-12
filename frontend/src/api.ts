import type { RobotStatus, Task } from './types'
import { ref } from 'vue'

const operatorToken = ref('')
export function setToken(value: string) { operatorToken.value = value }
export function hasToken() { return operatorToken.value.length > 0 }

async function request<T>(path: string, init: RequestInit = {}, write = false): Promise<T> {
  const headers: Record<string, string> = { 'Content-Type': 'application/json' }
  if (write) headers['X-Operator-Token'] = operatorToken.value
  const response = await fetch(path, { ...init, headers })
  const body = await response.json()
  if (!response.ok) throw new Error(body?.error?.message || `HTTP ${response.status}`)
  return body as T
}

export const getStatus = () => request<RobotStatus>('/api/v1/status')
export const getTask = (id: string) => request<Task>(`/api/v1/tasks/${id}`)
export const move = (target: number[], maxVelocity: number) =>
  request<{task_id: string; state: string}>('/api/v1/tasks/move', {
    method: 'POST', body: JSON.stringify({ target, max_velocity: maxVelocity }),
  }, true)
export const cancel = (id: string) => request(`/api/v1/tasks/${id}/cancel`, { method: 'POST' }, true)
export const estop = (engage: boolean) => request('/api/v1/estop', {
  method: 'POST', body: JSON.stringify({ engage }),
}, true)
export const resetFault = () => request('/api/v1/fault/reset', {
  method: 'POST', body: '{}',
}, true)
