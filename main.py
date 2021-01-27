# micropython-based footswitch using the moddevices Arduino shield
# attached to an STM32 Nucleo development board (NUCLEO-F-446RE)
import uasyncio as asyncio
from footswitch import Footswitch
import gc
import sys
sys.path.append('./')

# This file is a standard uasyncio event loop. The real
# magic (that a user would be interested in) is in footswitch.py

async def run_gc():
    gc.collect()
    gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
    asyncio.sleep(30)

def set_global_exception():
    def handle_exception(loop, context):
        import sys
        sys.print_exception(context["exception"])
        sys.exit()
    loop = asyncio.get_event_loop()
    loop.set_exception_handler(handle_exception)

async def main():
    set_global_exception()
    footswitch = Footswitch()
    gc.collect()
    # asyncio.create_task(run_gc())
    await footswitch.run()

try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()
