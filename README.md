# moveit_mpp

## Overview

A proof-of-concept *multi planner plugin planner plugin* for MoveIt.

## State

This is demo code at best. Use at your own risk. It will crash, it may do strange things.

It was just an experiment to see whether this approach could/would be a viable way of getting MoveIt to work with multiple planning plugins without extensive changes to the rest of its infrastructure.

## Known issues

While this can successfully load multiple MoveIt planning plugins, the rest of MoveIt does not know about this (or care). One example are the planning adapters: those are active for all planners (as those are configured per `move_group`) while that may not be needed or desirable.

## Roadmap

No further development of this code is currently planned. It is also not used in any systems that the author maintains.
