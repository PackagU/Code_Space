def build_dryrun_steps(mission):
    return [
        f"NavigateRoute:{mission.start_floor}:elevator_exit->{mission.pickup_point}",
        f"WaitForAck:{mission.mock_load_event}",
        f"NavigateRoute:{mission.start_floor}:{mission.pickup_point}->{mission.elevator_entry_point}",
        f"CallElevator:{mission.start_floor}",
        f"WaitElevatorArrived:{mission.start_floor}:open",
        f"NavigateRoute:{mission.start_floor}:{mission.elevator_entry_point}->{mission.elevator_inside_point}",
        f"CallElevator:{mission.target_floor}",
        f"SwitchFloor:{mission.target_floor}:{mission.elevator_inside_point}",
        f"WaitElevatorArrived:{mission.target_floor}:open",
        f"Relocalize:{mission.target_floor}:{mission.elevator_exit_point}",
        f"NavigateRoute:{mission.target_floor}:{mission.elevator_exit_point}",
        f"NavigateRoute:{mission.target_floor}:{mission.elevator_exit_point}->{mission.destination_point}",
        f"WaitForAck:{mission.mock_delivery_event}",
    ]
