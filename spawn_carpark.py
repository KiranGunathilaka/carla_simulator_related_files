#!/usr/bin/env python

# Copyright (c) 2025
# This script spawns parked vehicles in a carpark layout using CARLA Python API 0.9.16

"""
CARLA Carpark Vehicle Spawner

This script spawns vehicles along parking middle lines defined in a 2D array.
Vehicles spawn at height with physics enabled and will naturally fall and settle
on terrain. The script exits after spawning - CARLA's physics simulation handles
the rest automatically.

Input Array Format:
Each row: [start_location, end_location, side_type, num_vehicles, min_spacing, exclude_vehicles]
- start_location: carla.Location(x, y, z)
- end_location: carla.Location(x, y, z)
- side_type: "left", "right", or "both"
- num_vehicles: int (total vehicles for this line)
- min_spacing: float (minimum spacing in meters)
- exclude_vehicles: list (OPTIONAL) - list of vehicle keywords to exclude (e.g., ['truck', 'van'])

Vehicles are selected randomly from the available pool after filtering.
"""

import carla
import math
import random
import argparse


class CarparkVehicleSpawner:
    """Manages spawning of parked vehicles in a carpark layout."""
    
    def __init__(self, world, spawn_height=50.0, parking_offset=2.5):
        """
        Initialize the carpark vehicle spawner.
        
        Args:
            world: CARLA world object
            spawn_height: Height above terrain to spawn vehicles (meters)
            parking_offset: Perpendicular offset from middle line to parking spot (meters)
        """
        self.world = world
        self.spawn_height = spawn_height
        self.parking_offset = parking_offset
        self.vehicles = []
        
    def get_ground_height(self, location):
        """
        Raycast downward to find ground height at given x, y location.
        
        Args:
            location: carla.Location with x, y coordinates
            
        Returns:
            float: Ground height (z coordinate)
        """
        # Cast ray from high above downward
        start_location = carla.Location(location.x, location.y, location.z + 1000)
        end_location = carla.Location(location.x, location.y, location.z - 1000)
        
        # Cast ray and get first hit (ground)
        hits = self.world.cast_ray(start_location, end_location)
        
        if hits:
            # Return the z coordinate of the first hit (ground)
            return hits[0].location.z
        else:
            # Fallback: return original z if no hit found
            return location.z
    
    def compute_perpendicular_vector(self, start_loc, end_loc):
        """
        Compute perpendicular vector to the line (for parking offset).
        
        Args:
            start_loc: carla.Location of line start
            end_loc: carla.Location of line end
            
        Returns:
            tuple: (perpendicular_unit_vector, line_direction_unit_vector)
        """
        # Line direction vector
        line_vec = carla.Vector3D(
            end_loc.x - start_loc.x,
            end_loc.y - start_loc.y,
            end_loc.z - start_loc.z
        )
        
        # Normalize line direction
        line_length = math.sqrt(line_vec.x**2 + line_vec.y**2 + line_vec.z**2)
        if line_length < 0.001:
            # Degenerate case: start and end are same point
            return (carla.Vector3D(1, 0, 0), carla.Vector3D(1, 0, 0))
        
        line_dir = carla.Vector3D(
            line_vec.x / line_length,
            line_vec.y / line_length,
            line_vec.z / line_length
        )
        
        # Perpendicular vector in XY plane (rotate 90 degrees)
        # In CARLA: (x, y) -> (-y, x) for 90 degree rotation
        perp_vec = carla.Vector3D(-line_dir.y, line_dir.x, 0)
        
        # Normalize perpendicular vector
        perp_length = math.sqrt(perp_vec.x**2 + perp_vec.y**2)
        if perp_length > 0.001:
            perp_vec = carla.Vector3D(
                perp_vec.x / perp_length,
                perp_vec.y / perp_length,
                0
            )
        else:
            perp_vec = carla.Vector3D(1, 0, 0)
        
        return (perp_vec, line_dir)
    
    def generate_spawn_positions(self, start_loc, end_loc, side_type, 
                                 num_vehicles, min_spacing):
        """
        Generate vehicle spawn positions along a parking middle line.
        
        Args:
            start_loc: carla.Location of line start
            end_loc: carla.Location of line end
            side_type: "left", "right", or "both"
            num_vehicles: Number of vehicles to spawn
            min_spacing: Minimum spacing between vehicles (meters)
            
        Returns:
            list: List of (carla.Transform, side) tuples for vehicle spawns
        """
        spawn_transforms = []
        
        # Compute line direction and perpendicular vectors
        perp_vec, line_dir = self.compute_perpendicular_vector(start_loc, end_loc)
        
        # Determine which sides to park on
        sides_to_use = []
        if side_type == "left":
            sides_to_use = ["left"]
        elif side_type == "right":
            sides_to_use = ["right"]
        elif side_type == "both":
            # Non-symmetrical: randomly assign vehicles to left or right
            sides_to_use = ["left", "right"]
        else:
            raise ValueError(f"Invalid side_type: {side_type}. Must be 'left', 'right', or 'both'")
        
        # Calculate line length
        line_length = math.sqrt(
            (end_loc.x - start_loc.x)**2 + 
            (end_loc.y - start_loc.y)**2 + 
            (end_loc.z - start_loc.z)**2
        )
        
        # Calculate maximum number of vehicles that fit
        max_vehicles = int(line_length / min_spacing)
        if num_vehicles > max_vehicles:
            print(f"Warning: Requested {num_vehicles} vehicles but only {max_vehicles} fit. "
                  f"Spawning {max_vehicles} vehicles.")
            num_vehicles = max_vehicles
        
        # Generate random spacing positions
        # Create a list of potential positions with random spacing
        positions = []
        current_pos = 0.0
        
        for i in range(num_vehicles):
            # Random spacing: min_spacing to 2*min_spacing (non-uniform)
            spacing = min_spacing + random.uniform(0, min_spacing)
            
            if current_pos + spacing > line_length:
                break
            
            positions.append(current_pos)
            current_pos += spacing
        
        # If we have fewer positions than requested, add more with tighter spacing
        while len(positions) < num_vehicles and current_pos < line_length:
            spacing = min_spacing + random.uniform(0, min_spacing * 0.5)
            if current_pos + spacing <= line_length:
                positions.append(current_pos)
                current_pos += spacing
            else:
                break
        
        # Create spawn transforms for each position
        for pos_along_line in positions:
            # Choose side (for "both", randomly assign)
            side = random.choice(sides_to_use) if len(sides_to_use) > 1 else sides_to_use[0]
            
            # Calculate position along line
            t = pos_along_line / line_length if line_length > 0 else 0
            line_pos = carla.Location(
                start_loc.x + t * (end_loc.x - start_loc.x),
                start_loc.y + t * (end_loc.y - start_loc.y),
                start_loc.z + t * (end_loc.z - start_loc.z)
            )
            
            # Offset perpendicular to line
            offset_direction = 1 if side == "right" else -1
            parking_pos = carla.Location(
                line_pos.x + offset_direction * self.parking_offset * perp_vec.x,
                line_pos.y + offset_direction * self.parking_offset * perp_vec.y,
                line_pos.z
            )
            
            # Get ground height at parking position
            ground_z = self.get_ground_height(parking_pos)
            parking_pos.z = ground_z + self.spawn_height
            
            # Calculate base yaw (rotation) - vehicle should be parallel to line
            base_yaw = math.degrees(math.atan2(line_dir.y, line_dir.x))
            
            # Rotate based on parking side:
            # Left side: +90 degrees (perpendicular, facing away from line)
            # Right side: -90 degrees (perpendicular, facing away from line)
            if side == "left":
                yaw = base_yaw + 90.0
            elif side == "right":
                yaw = base_yaw - 90.0
            else:
                # For "both" case, use the randomly assigned side
                yaw = base_yaw + 90.0 if side == "left" else base_yaw - 90.0
            
            # Add slight random rotation variation (±2 degrees for more realistic parking)
            yaw += random.uniform(-2, 2)
            
            spawn_transform = carla.Transform(
                parking_pos,
                carla.Rotation(yaw=yaw, pitch=0, roll=0)
            )
            
            spawn_transforms.append((spawn_transform, side))
        
        return spawn_transforms
    
    def filter_vehicles_for_line(self, vehicle_blueprints, exclude_keywords=None):
        """
        Filter vehicles for a specific parking line based on exclude keywords.
        
        Args:
            vehicle_blueprints: List of vehicle blueprints
            exclude_keywords: List of keywords to exclude (e.g., ['truck', 'van'])
            
        Returns:
            list: Filtered vehicle blueprints
        """
        if exclude_keywords is None:
            exclude_keywords = []
        
        filtered = []
        for bp in vehicle_blueprints:
            bp_id_lower = bp.id.lower()
            is_excluded = any(keyword.lower() in bp_id_lower for keyword in exclude_keywords)
            
            if not is_excluded:
                filtered.append(bp)
        
        return filtered
    
    def spawn_vehicles_along_line(self, parking_line_data, vehicle_blueprints):
        """
        Spawn vehicles along a single parking middle line.
        
        Args:
            parking_line_data: [start_loc, end_loc, side_type, num_vehicles, min_spacing, exclude_vehicles]
                exclude_vehicles is optional and can be a list of vehicle keywords to exclude
            vehicle_blueprints: List of vehicle blueprints to choose from
            
        Returns:
            list: List of spawned vehicle actors
        """
        # Parse parking line data (support optional exclude_vehicles)
        if len(parking_line_data) >= 6:
            start_loc, end_loc, side_type, num_vehicles, min_spacing, exclude_vehicles = parking_line_data
        else:
            start_loc, end_loc, side_type, num_vehicles, min_spacing = parking_line_data
            exclude_vehicles = None
        
        # Filter vehicles for this line
        line_vehicle_blueprints = self.filter_vehicles_for_line(vehicle_blueprints, exclude_vehicles)
        
        if not line_vehicle_blueprints:
            print(f"Warning: No vehicles available after filtering. Skipping this parking line.")
            return []
        
        # Generate spawn positions
        spawn_transforms = self.generate_spawn_positions(
            start_loc, end_loc, side_type, num_vehicles, min_spacing
        )
        
        spawned_vehicles = []
        
        for transform, side in spawn_transforms:
            # Randomly select a vehicle blueprint from filtered list
            vehicle_bp = random.choice(line_vehicle_blueprints)
            
            try:
                # Spawn vehicle
                vehicle = self.world.spawn_actor(vehicle_bp, transform)
                
                # Ensure physics and gravity are enabled
                vehicle.set_simulate_physics(True)
                vehicle.set_enable_gravity(True)
                
                # Set initial velocity to zero (will fall due to gravity)
                vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
                
                # Apply brakes and handbrake to prevent sliding after landing
                control = carla.VehicleControl()
                control.brake = 0.5  # Full brake
                control.hand_brake = False  # Handbrake engaged
                control.throttle = 0.0
                control.steer = 0.0
                vehicle.apply_control(control)
                
                spawned_vehicles.append(vehicle)
                print(f"Spawned {vehicle_bp.id} at ({transform.location.x:.2f}, "
                      f"{transform.location.y:.2f}, {transform.location.z:.2f}) on {side} side "
                      f"(yaw: {transform.rotation.yaw:.1f}°)")
                
            except Exception as e:
                print(f"Failed to spawn vehicle: {e}")
        
        return spawned_vehicles
    
    def filter_vehicles(self, vehicle_blueprints):
        """
        Filter out bikes and 2-wheeled vehicles.
        
        Args:
            vehicle_blueprints: List of vehicle blueprints
            
        Returns:
            list: Filtered vehicle blueprints (4-wheeled vehicles only)
        """
        excluded_keywords = ['bike', 'bicycle', 'motorcycle', 'gazelle', 'harley', 'yamaha', 
                            'kawasaki', 'ninja', 'vespa', 'diamondback', 'bh.crossbike', 'european_hgv', 'firetruck' , 'fusorosa', 'rad']
        
        filtered = []
        excluded_count = 0
        
        for bp in vehicle_blueprints:
            bp_id_lower = bp.id.lower()
            is_excluded = any(keyword in bp_id_lower for keyword in excluded_keywords)
            
            if not is_excluded:
                filtered.append(bp)
            else:
                excluded_count += 1
        
        if excluded_count > 0:
            print(f"Filtered out {excluded_count} bike/2-wheeled vehicles")
        
        return filtered
    
    def spawn_carpark(self, parking_lines, vehicle_filter='vehicle.*'):
        """
        Spawn vehicles for all parking lines defined in the input array.
        
        Args:
            parking_lines: List of parking line data arrays
            vehicle_filter: Blueprint filter string for vehicles
            
        Returns:
            list: All spawned vehicle actors
        """
        # Get vehicle blueprints
        blueprint_library = self.world.get_blueprint_library()
        vehicle_blueprints = blueprint_library.filter(vehicle_filter)
        
        if not vehicle_blueprints:
            raise ValueError(f"No vehicle blueprints found matching filter: {vehicle_filter}")
        
        # Filter out bikes and 2-wheeled vehicles
        vehicle_blueprints = self.filter_vehicles(vehicle_blueprints)
        
        if not vehicle_blueprints:
            raise ValueError("No 4-wheeled vehicles found after filtering out bikes/2-wheeled vehicles")
        
        print(f"Found {len(vehicle_blueprints)} vehicle blueprints (4-wheeled vehicles only)")
        print(f"Processing {len(parking_lines)} parking lines...\n")
        
        # Spawn vehicles for each parking line
        for i, parking_line in enumerate(parking_lines):
            print(f"Processing parking line {i+1}/{len(parking_lines)}...")
            
            # Show exclude filters if present
            if len(parking_line) >= 6 and parking_line[5]:
                exclude_list = parking_line[5]
                print(f"  Excluding vehicles: {exclude_list}")
            
            vehicles = self.spawn_vehicles_along_line(parking_line, vehicle_blueprints)
            self.vehicles.extend(vehicles)
            print(f"  Spawned {len(vehicles)} vehicles\n")
        
        print(f"Total vehicles spawned: {len(self.vehicles)}\n")
        
        return self.vehicles
    
    def cleanup(self):
        """Destroy all spawned vehicles."""
        print(f"Cleaning up {len(self.vehicles)} vehicles...")
        for vehicle in self.vehicles:
            try:
                vehicle.destroy()
            except:
                pass
        self.vehicles.clear()


def main():
    """Main function demonstrating carpark vehicle spawning."""
    
    # Parse command line arguments
    argparser = argparse.ArgumentParser(
        description='CARLA Carpark Vehicle Spawner - Spawns parked vehicles in a carpark layout')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port of CARLA Simulator (default: 2000)')
    args = argparser.parse_args()
    
    # Connect to CARLA
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    
    try:
        # ====================================================================
        # INPUT DATA
        # ====================================================================
        # Format: [start_location, end_location, side_type, num_vehicles, min_spacing, exclude_vehicles]
        #   - exclude_vehicles is OPTIONAL (6th element)
        #   - exclude_vehicles: list of vehicle keywords to exclude (e.g., ['truck', 'van', 'audi'])
        #   - side_type: "left", "right", or "both"
        #   - Vehicles are selected sequentially (not randomly) from available pool
        
        parking_lines = [
            # Example parking line 1: Left side, 14 vehicles, 5m spacing
            # No exclude filter - uses all available vehicles
            [
                carla.Location(x=302.0, y=-226.1, z=0),  # Start
                carla.Location(x=383.2, y=-226.10, z=0),  # End
                "left",      # Side type
                14,          # Number of vehicles
                5.0,         # Minimum spacing (meters)
                []           # Exclude vehicles (empty = use all)
            ],
            
            #2 - Example with exclude filter: exclude trucks and vans
            [
                carla.Location(x=302.0, y=-221.6, z=0),  # Start
                carla.Location(x=383.2, y=-221.6, z=0),  # End
                "right",      # Side type
                10,          # Number of vehicles
                8.0,         # Minimum spacing (meters)
            ],
            
            #3 - Both sides, no exclude filter
            [
                carla.Location(x=386.2, y=-202.5, z=0),  # Start
                carla.Location(x=290.9, y=-202.5, z=0),  # End
                "both",      # Side type
                30,          # Number of vehicles
                4.0          # Minimum spacing (meters)
                # No exclude filter - can omit 6th element
            ],
            
            #4
            [
                carla.Location(x=386.2, y=-184.0, z=0),  # Start
                carla.Location(x=290.9, y=-184.0, z=0),  # End
                "both",      # Side type
                30,          # Number of vehicles
                4.0          # Minimum spacing (meters)
            ],
            
            #5
            [
                carla.Location(x=382.2, y=-165.4, z=0),  # Start
                carla.Location(x=290.9, y=-165.4, z=0),  # End
                "both",      # Side type
                30,          # Number of vehicles
                4.0          # Minimum spacing (meters)
            ],
            
            #6
            [
                carla.Location(x=383.7, y=-146.6, z=0),  # Start
                carla.Location(x=301.8, y=-146.6, z=0),  # End
                "right",      # Side type
                15,          # Number of vehicles
                10.0          # Minimum spacing (meters)
            ],
            
            #7
            [
                carla.Location(x=383.7, y=-142.6, z=0),  # Start
                carla.Location(x=301.8, y=-142.6, z=0),  # End
                "left",      # Side type
                12,          # Number of vehicles
                8.0          # Minimum spacing (meters)
            ],
            
            #8
            [
                carla.Location(x=370.7, y=-123.50, z=0),  # Start
                carla.Location(x=301.8, y=-123.50, z=0),  # End
                "right",      # Side type
                16,          # Number of vehicles
                6.0          # Minimum spacing (meters)
            ],
            
            #9
            [
                carla.Location(x=402.9, y=-133.40, z=0),  # Start
                carla.Location(x=402.9, y=-235.40, z=0),  # End
                "left",      # Side type
                20,          # Number of vehicles
                4.0          # Minimum spacing (meters)
            ],
            
            #10   car park2 
            [
                carla.Location(x=326.3, y=-109.1, z=0),  # Start
                carla.Location(x=274.7, y=-109.1, z=0),  # End
                "both",      # Side type
                20,          # Number of vehicles
                4.0,          # Minimum spacing (meters)
                ['truck', 'van', 'carlacola']  # Exclude vehicles containing 'truck' or 'van'
            ],
            
            #11
            [
                carla.Location(x=329.9, y=-91.90, z=0),  # Start
                carla.Location(x=276.1, y=-91.90, z=0),  # End
                "both",      # Side type
                20,          # Number of vehicles
                4.0,          # Minimum spacing (meters)
                ['truck', 'van', 'carlacola']  # Exclude vehicles containing 'truck' or 'van'
            ],
            
            #12
            [
                carla.Location(x=326.1, y=-73.20, z=0),  # Start
                carla.Location(x=274.1, y=-73.20, z=0),  # End
                "both",      # Side type
                20,          # Number of vehicles
                4.0,          # Minimum spacing (meters)
                ['truck', 'van', 'carlacola']    
            ], 
            
            #13
            [
                carla.Location(x=326.1, y=-53.40, z=0),  # Start
                carla.Location(x=274.1, y=-53.40, z=0),  # End
                "both",      # Side type
                20,          # Number of vehicles
                4.0,          # Minimum spacing (meters)
                ['truck', 'van', 'carlacola']    
            ], 
            
            #14
            [
                carla.Location(x=326.1, y=-33.20, z=0),  # Start
                carla.Location(x=274.1, y=-33.20, z=0),  # End
                "both",      # Side type
                20,          # Number of vehicles
                4.0,          # Minimum spacing (meters)
                ['truck', 'van', 'carlacola']    
            ], 
            
            #15   car park4 
            [
                carla.Location(x=100.9, y=-238.5, z=0),  # Start
                carla.Location(x=52.0, y=-238.5, z=0),  # End
                "both",      # Side type
                10,          # Number of vehicles
                4.0,          # Minimum spacing (meters)
                ['truck', 'van', 'carlacola']  # Exclude vehicles containing 'truck' or 'van'
            ],
            
            #16
            [
                carla.Location(x=100.9, y=-219.7, z=0),  # Start
                carla.Location(x=52.0, y=-219.7, z=0),  # End
                "both",      # Side type
                10,          # Number of vehicles
                4.0,          # Minimum spacing (meters)
                ['truck', 'van', 'carlacola']  # Exclude vehicles containing 'truck' or 'van'
            ],
            
            #17
            [
                carla.Location(x=100.9, y=-201.3, z=0),  # Start
                carla.Location(x=52.0, y=-201.3, z=0),  # End
                "both",      # Side type
                10,          # Number of vehicles
                4.0,          # Minimum spacing (meters)
                ['truck', 'van', 'carlacola']  # Exclude vehicles containing 'truck' or 'van'
            ],
            
            #18 Car park 3
            [
                carla.Location(x=129.6, y=-80.0, z=0),  # Start
                carla.Location(x=48.4, y=-80.3, z=0),  # End
                "both",      # Side type
                10,          # Number of vehicles
                4.0,          # Minimum spacing (meters)
                ['truck', 'van', 'carlacola']  # Exclude vehicles containing 'truck' or 'van'
            ],
            
            #19
            [
                carla.Location(x=129.6, y=-62.0, z=0),  # Start
                carla.Location(x=48.4, y=-62.0, z=0),  # End
                "both",      # Side type
                10,          # Number of vehicles
                4.0,          # Minimum spacing (meters)
                ['truck', 'van', 'carlacola']  # Exclude vehicles containing 'truck' or 'van'
            ],
            
            #20
            [
                carla.Location(x=129.6, y=-44.8, z=0),  # Start
                carla.Location(x=48.4, y=-44.8, z=0),  # End
                "both",      # Side type
                10,          # Number of vehicles
                4.0,          # Minimum spacing (meters)
                ['truck', 'van', 'carlacola']  # Exclude vehicles containing 'truck' or 'van'
            ],
            
            
        ]
        
        # ====================================================================
        # SPAWN CARPARK VEHICLES
        # ====================================================================
        
        # Create spawner
        spawner = CarparkVehicleSpawner(
            world=world,
            spawn_height=1.5,        # Spawn 50m above ground
            parking_offset=3.0         # 2.5m offset from middle line
        )
        
        # Spawn all vehicles
        vehicles = spawner.spawn_carpark(
            parking_lines=parking_lines,
            vehicle_filter='vehicle.*'  # Use all vehicle types
        )
        
        print(f"\nSuccessfully spawned {len(vehicles)} vehicles with physics enabled.")
        print("Vehicles will fall and settle naturally in CARLA's physics simulation.")
        print("Script exiting - vehicles remain in the world as physics-enabled objects.\n")
    
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

