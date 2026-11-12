import asyncio
import json
import sys

import pytest

from app.process_adapter import ProcessRosAdapter
from app.ros_adapter import AdapterError


@pytest.mark.skipif(sys.platform == 'win32', reason='Unix socket bridge runs on Ubuntu')
def test_bridge_rpc_and_offline_detection(tmp_path):
    async def run():
        adapter = ProcessRosAdapter()
        adapter._socket = str(tmp_path / 'bridge.sock')

        async def handle(reader, writer):
            request = json.loads(await reader.readline())
            if request['method'] == 'snapshot':
                result = {'status': dict(adapter._state, mode='IDLE', age_ms=12), 'tasks': {}}
                response = {'result': result}
            else:
                response = {'error': {'code': 'GOAL_REJECTED', 'status': 409, 'message': 'rejected'}}
            writer.write((json.dumps(response) + '\n').encode())
            await writer.drain()
            writer.close()
            await writer.wait_closed()

        server = await asyncio.start_unix_server(handle, adapter._socket)
        try:
            async with server:
                await adapter._refresh()
                assert adapter.status()['mode'] == 'IDLE'
                with pytest.raises(AdapterError) as failure:
                    await adapter._rpc('move')
                assert failure.value.status == 409
                adapter._last_poll -= 3
                assert adapter.status()['mode'] == 'OFFLINE'
        finally:
            server.close()
            await server.wait_closed()

    asyncio.run(run())
