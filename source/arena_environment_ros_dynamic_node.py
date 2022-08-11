#!/usr/bin/env python3

from arena_environment_ros_d import ArenaObstaclesROS


def main():
    arena_obst = ArenaObstaclesROS()
    arena_obst.begin()
    arena_obst.run()


if __name__ == "__main__":
    main()
