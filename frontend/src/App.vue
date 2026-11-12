<script setup lang="ts">
import { computed, onMounted, onUnmounted, ref } from 'vue'
import { cancel, estop, getStatus, getTask, hasToken, move, resetFault, setToken } from './api'
import type { RobotStatus, Task } from './types'

const offline: RobotStatus = {
  mode: 'OFFLINE', fault_code: 'NONE', sensor_fresh: false,
  position: [0, 0], target: [0, 0], active_task_id: '', progress: 0,
  observed_at: '', age_ms: null,
}
const status = ref<RobotStatus>(offline)
const task = ref<Task | null>(null)
const target1 = ref(0.5)
const target2 = ref(-0.5)
const speed = ref(0.3)
const tokenInput = ref('')
const notice = ref('')
const busy = ref(false)
const connected = ref(false)
let socket: WebSocket | null = null
let pollTimer: number | undefined
let reconnectTimer: number | undefined
let lastMessage = 0

const canMove = computed(() => connected.value && status.value.mode === 'IDLE' && hasToken() && !busy.value)
const canCancel = computed(() => task.value?.state === 'RUNNING' && hasToken() && !busy.value)
const modeLabel = computed(() => ({
  STARTING: '启动中', IDLE: '待命', RUNNING: '运行中', FAULT: '故障锁存',
  ESTOP: '软件急停', OFFLINE: '控制节点离线',
}[status.value.mode]))

function saveToken() {
  setToken(tokenInput.value.trim())
  tokenInput.value = ''
  notice.value = hasToken() ? '操作口令已保存在当前页面内存中。' : '请输入操作口令。'
}

async function perform(action: () => Promise<unknown>, success: string) {
  busy.value = true
  notice.value = ''
  try {
    await action()
    notice.value = success
  } catch (error) {
    notice.value = error instanceof Error ? error.message : '操作失败'
  } finally {
    busy.value = false
  }
}

function sendMove() {
  perform(async () => {
    const response = await move([target1.value, target2.value], speed.value)
    task.value = await getTask(response.task_id)
  }, '任务已被控制节点接受。')
}

function sendCancel() {
  if (!task.value) return
  perform(() => cancel(task.value!.task_id), '取消请求已被控制节点接受，等待任务结束。')
}

function connect() {
  const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:'
  socket = new WebSocket(`${protocol}//${location.host}/ws/v1/status`)
  socket.onmessage = (event) => {
    try {
      status.value = JSON.parse(event.data) as RobotStatus
      connected.value = true
      lastMessage = Date.now()
    } catch { notice.value = '状态数据格式错误。' }
  }
  socket.onclose = () => {
    connected.value = false
    status.value = offline
    reconnectTimer = window.setTimeout(connect, 1500)
  }
  socket.onerror = () => socket?.close()
}

onMounted(async () => {
  try { status.value = await getStatus() } catch { status.value = offline }
  connect()
  pollTimer = window.setInterval(async () => {
    if (Date.now() - lastMessage > 2500) {
      connected.value = false
      status.value = offline
      if (socket?.readyState === WebSocket.OPEN) socket.close()
    }
    if (task.value && ['RUNNING', 'CANCEL_REQUESTED'].includes(task.value.state)) {
      try { task.value = await getTask(task.value.task_id) } catch { /* status remains visible */ }
    }
  }, 500)
})
onUnmounted(() => {
  if (pollTimer) clearInterval(pollTimer)
  if (reconnectTimer) clearTimeout(reconnectTimer)
  if (socket) { socket.onclose = null; socket.close() }
})
</script>

<template>
  <main class="shell">
    <header>
      <div><div class="eyebrow">ROBOT SYSTEM DEMO · ROS 2 JAZZY</div><h1>RobotOS-Lite</h1>
      <p>两轴仿真机器人的控制与状态监控</p></div>
      <div class="badge" :class="status.mode.toLowerCase()">{{ modeLabel }}</div>
    </header>

    <div class="warning">仅用于软件仿真演示。页面“急停”是软件命令，不是实体机器人安全回路。控制节点离线时无法确认命令送达。</div>

    <section class="grid">
      <article class="card state">
        <h2>实时状态</h2>
        <div class="metric"><span>控制状态</span><strong>{{ modeLabel }}</strong></div>
        <div class="metric"><span>传感器数据</span><strong>{{ status.sensor_fresh ? '正常' : '不可用' }}</strong></div>
        <div class="metric"><span>故障码</span><strong>{{ status.fault_code }}</strong></div>
        <div class="metric"><span>状态年龄</span><strong>{{ status.age_ms === null ? '—' : `${status.age_ms} ms` }}</strong></div>
        <div class="progress"><div :style="{ width: `${Math.round(status.progress * 100)}%` }"></div></div>
        <small>任务进度 {{ Math.round(status.progress * 100) }}%</small>
      </article>

      <article class="card joints">
        <h2>关节角度 <small>rad</small></h2>
        <div v-for="index in 2" :key="index" class="joint">
          <div class="joint-title"><span>J0{{ index }}</span><strong>{{ status.position[index - 1]?.toFixed(3) }}</strong></div>
          <div class="track"><div :style="{ left: `${((status.position[index - 1] + 1.57) / 3.14) * 100}%` }"></div></div>
          <small>目标 {{ status.target[index - 1]?.toFixed(3) }} · 限位 ±1.57</small>
        </div>
      </article>

      <article class="card controls">
        <h2>任务控制</h2>
        <div class="form-row"><label>关节 1 目标<input v-model.number="target1" type="number" step="0.1" min="-1.57" max="1.57"></label>
        <label>关节 2 目标<input v-model.number="target2" type="number" step="0.1" min="-1.57" max="1.57"></label></div>
        <label>最大速度 (rad/s)<input v-model.number="speed" type="number" step="0.1" min="0.01" max="0.5"></label>
        <div class="buttons"><button class="primary" :disabled="!canMove" @click="sendMove">发送移动任务</button>
        <button :disabled="!canCancel" @click="sendCancel">取消任务</button></div>
        <div class="buttons"><button class="danger" :disabled="busy || !hasToken()" @click="perform(() => estop(true), '软件急停已由控制节点确认。')">软件急停</button>
        <button :disabled="busy || !hasToken()" @click="perform(() => estop(false), '急停解除请求已确认。')">解除急停</button>
        <button :disabled="busy || !hasToken()" @click="perform(resetFault, '故障复位已确认。')">故障复位</button></div>
      </article>

      <article class="card session">
        <h2>操作会话</h2>
        <p>写操作需要在启动后端时设置的口令；仅保存在本页内存。</p>
        <div class="token"><input v-model="tokenInput" type="password" placeholder="输入操作口令" @keyup.enter="saveToken"><button @click="saveToken">启用操作</button></div>
        <div class="task" v-if="task"><span>最近任务</span><strong>{{ task.state }}</strong><code>{{ task.task_id }}</code><small>{{ task.code || '进行中' }}</small></div>
        <p class="notice" aria-live="polite">{{ notice }}</p>
      </article>
    </section>
    <footer>RobotOS-Lite · 本地仿真原型 · 状态与位置均来自 ROS 2</footer>
  </main>
</template>
