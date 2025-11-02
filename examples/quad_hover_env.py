from target.simulator import Simulator

if __name__ == '__main__':
    import os
    RENDER = int(os.getenv('RENDER', '1'))
    render_mode = 'human' if RENDER else None
    sim = Simulator(render_mode=render_mode)
    sim.run() 