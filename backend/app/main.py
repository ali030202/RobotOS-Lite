import asyncio
import hmac
import os
from contextlib import asynccontextmanager
from pathlib import Path
from uuid import uuid4
from typing import Optional

from fastapi import Depends, FastAPI, Header, Request, WebSocket
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

from .ros_adapter import AdapterError, RosAdapter
from .process_adapter import ProcessRosAdapter
from .schemas import EStopRequest, MoveRequest


def create_app(adapter=None):
    @asynccontextmanager
    async def lifespan(app):
        app.state.adapter = adapter or (
            ProcessRosAdapter() if os.getenv('ROBOTOS_ROS_PYTHON') else RosAdapter())
        if adapter is None:
            await app.state.adapter.start()
        yield
        if adapter is None:
            await app.state.adapter.stop()

    app = FastAPI(title='RobotOS-Lite', lifespan=lifespan)

    def error(code, message, status):
        return JSONResponse(status_code=status, content={
            'error': {'code': code, 'message': message, 'request_id': str(uuid4())}})

    @app.exception_handler(AdapterError)
    async def adapter_error(_request: Request, exc: AdapterError):
        return error(exc.code, exc.message, exc.status)

    @app.exception_handler(RequestValidationError)
    async def validation_error(_request: Request, _exc: RequestValidationError):
        return error('INVALID_INPUT', '请求参数不符合接口约定', 422)

    async def require_token(x_operator_token: Optional[str] = Header(default=None)):
        expected = os.getenv('ROBOTOS_OPERATOR_TOKEN', '')
        if not expected or not x_operator_token or not hmac.compare_digest(x_operator_token, expected):
            raise AdapterError('UNAUTHORIZED', 401, '操作口令无效')

    @app.get('/healthz')
    def health(request: Request):
        return {'api': 'ok', 'ros_connected': request.app.state.adapter.status()['mode'] != 'OFFLINE'}

    @app.get('/api/v1/status')
    def status(request: Request):
        return request.app.state.adapter.status()

    @app.post('/api/v1/tasks/move', status_code=202, dependencies=[Depends(require_token)])
    async def move(body: MoveRequest, request: Request):
        return await request.app.state.adapter.move(body.target, body.max_velocity)

    @app.get('/api/v1/tasks/{task_id}')
    def task(task_id: str, request: Request):
        return request.app.state.adapter.task(task_id)

    @app.post('/api/v1/tasks/{task_id}/cancel', status_code=202, dependencies=[Depends(require_token)])
    async def cancel(task_id: str, request: Request):
        return await request.app.state.adapter.cancel(task_id)

    @app.post('/api/v1/estop', dependencies=[Depends(require_token)])
    async def estop(body: EStopRequest, request: Request):
        return await request.app.state.adapter.estop(body.engage)

    @app.post('/api/v1/fault/reset', dependencies=[Depends(require_token)])
    async def reset(request: Request):
        return await request.app.state.adapter.reset_fault()

    @app.websocket('/ws/v1/status')
    async def ws_status(websocket: WebSocket):
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(websocket.app.state.adapter.status())
                await asyncio.sleep(0.2)
        except Exception:
            pass

    dist = Path(__file__).resolve().parents[2] / 'frontend' / 'dist'
    if dist.exists():
        app.mount('/', StaticFiles(directory=dist, html=True), name='frontend')
    return app


app = create_app()
