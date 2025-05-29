# Courtesy of Google AI in reponse to:
# Write a class using asyncio and evdev to report gamepad joystick values

import asyncio
from evdev import InputDevice, categorize, ecodes

class AsyncGamepad:
    """An asynchronous gamepad reader using evdev."""
    def __init__(self, device_path):
        self.device = InputDevice(device_path)
        self.axis_states = {} # Stores latest values for each axis
        self.running = False

        # Mapping from evdev axis codes to descriptive names
        self.axis_map = {
            ecodes.ABS_X: 'left_stick_x',
            ecodes.ABS_Y: 'left_stick_y',
            ecodes.ABS_RX: 'right_stick_x',
            ecodes.ABS_RY: 'right_stick_y',
            }

    async def read_events(self):
        """Reads and processes input events from the gamepad device."""
        async for event in self.device.async_read_loop():
            if event.type == ecodes.EV_ABS and event.code in self.axis_map:
                axis_name = self.axis_map[event.code]
                self.axis_states[axis_name] = event.value
                #print(f"Axis {axis_name}: {event.value}")
                #print(self.axis_states)

    async def start(self):
        """Starts the event reading loop."""
        if not self.running:
            self.running = True
            await self.read_events()

    async def stop(self):
        """Stops the event reading loop."""
        self.running = False

    def get_axis_state(self, axis_name):
        """Returns latest value for axis_name, or None if not yet read."""
        return self.axis_states.get(axis_name)
