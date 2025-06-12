import asyncio
from js_client import JS_Client

jsc = JS_Client()

async def main():
    js_task = asyncio.create_task(jsc.start())
    while True:
        js_vals = jsc.get_js_vals()
        if js_vals:
            print(js_vals)
        await asyncio.sleep(0.1)

asyncio.run(main())
