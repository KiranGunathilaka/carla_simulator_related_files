#!/usr/bin/env python

# Copyright (c) 2025
# This script spawns parked vehicles in a carpark layout using CARLA Python API 0.9.16

"""
CARLA Carpark Vehicle Spawner

This script spawns vehicles along parking middle lines defined in a 2D array.
By default, uses static.prop.mesh for better performance (no vehicle component overhead).
Vehicles spawn at height with physics enabled and will naturally fall and settle
on terrain. The script exits after spawning - CARLA's physics simulation handles
the rest automatically.

Performance:
- Uses static.prop.mesh with mass attribute (better performance, same as manual drag-drop)
- Falls back to vehicle blueprints if mesh paths not found
- Physics automatically enabled when mass > 0

Input Array Format:
Each row: [start_location, end_location, side_type, num_vehicles, min_spacing, exclude_vehicles]
- start_location: carla.Location(x, y, z)
- end_location: carla.Location(x, y, z)
- side_type: "left", "right", or "both"
- num_vehicles: int (total vehicles for this line)
- min_spacing: float (minimum spacing in meters)
- exclude_vehicles: list (OPTIONAL) - list of vehicle keywords to exclude (e.g., ['truck', 'van'])

Vehicles are selected randomly from the available pool after filtering.

Note: To use static props, you may need to add vehicle mesh path mappings in get_vehicle_mesh_path().
"""

import carla
import math
import random
import argparse
import json
import os


class CarparkVehicleSpawner:
    """Manages spawning of parked vehicles in a carpark layout using static props for performance."""
    
    def __init__(self, world, spawn_height=50.0, parking_offset=2.5, use_static_props=True, prop_mass=100.0):
        """
        Initialize the carpark vehicle spawner.
        
        Args:
            world: CARLA world object
            spawn_height: Height above terrain to spawn vehicles (meters)
            parking_offset: Perpendicular offset from middle line to parking spot (meters)
            use_static_props: If True, use static.prop.mesh (better performance). If False, use vehicle blueprints.
            prop_mass: Mass for static props when physics enabled (kg)
        """
        self.world = world
        self.spawn_height = spawn_height
        self.parking_offset = parking_offset
        self.use_static_props = use_static_props
        self.prop_mass = prop_mass
        self.vehicles = []
        self.static_prop_bp = None
        
        # Initialize static prop blueprint if using static props
        if self.use_static_props:
            blueprint_library = self.world.get_blueprint_library()
            try:
                self.static_prop_bp = blueprint_library.find('static.prop.mesh')
                if self.static_prop_bp is None:
                    print("Warning: static.prop.mesh not found. Falling back to vehicle blueprints.")
                    self.use_static_props = False
            except:
                print("Warning: Could not find static.prop.mesh. Falling back to vehicle blueprints.")
                self.use_static_props = False
        
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
    
    def get_vehicle_mesh_path(self, vehicle_bp_id):
        """
        Get static mesh path for a vehicle blueprint ID.
        
        This function attempts to construct mesh paths based on vehicle ID patterns.
        You can extend this mapping with actual mesh paths from CARLA content browser.
        
        To find mesh paths:
        1. Open CARLA in Unreal Editor
        2. Content Browser -> Search for vehicle meshes
        3. Right-click mesh -> Copy Reference
        4. Add mapping below
        
        Args:
            vehicle_bp_id: Vehicle blueprint ID (e.g., 'vehicle.audi.a2')
            
        Returns:
            str or None: Mesh path if found, None otherwise (falls back to vehicle blueprint)
        """
        # Vehicle blueprint ID -> static mesh path mapping
        # Based on available parked vehicle meshes in /Game/Carla/Static/Car/4Wheeled/
        # Priority: ParkedVehicles folder > Individual vehicle folders with parked meshes > Regular meshes
        VEHICLE_MESH_MAPPING = {
            # ===== ParkedVehicles folder (dedicated parked meshes) =====
            'vehicle.dodge.charger': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/Charger/SM_ChargerParked.SM_ChargerParked',
            'vehicle.dodge.charger_2020': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/Charger/SM_ChargerParked.SM_ChargerParked',
            'vehicle.dodge.charger_police': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/Charger/SM_ChargerParked.SM_ChargerParked',
            'vehicle.dodge.charger_police_2020': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/Charger/SM_ChargerParked.SM_ChargerParked',
            'vehicle.ford.crown': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/FordCrown/SM_FordCrown_parked.SM_FordCrown_parked',
            'vehicle.lincoln.mkz_2017': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/Lincoln/SM_LincolnParked.SM_LincolnParked',
            'vehicle.lincoln.mkz_2020': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/Lincoln/SM_LincolnParked.SM_LincolnParked',
            'vehicle.mercedes.coupe': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/MercedesCCC/SM_MercedesCCC_Parked.SM_MercedesCCC_Parked',
            'vehicle.mercedes.coupe_2020': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/MercedesCCC/SM_MercedesCCC_Parked.SM_MercedesCCC_Parked',
            'vehicle.mini.cooper_s': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/Mini2021/SM_Mini2021_parked.SM_Mini2021_parked',
            'vehicle.mini.cooper_s_2021': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/Mini2021/SM_Mini2021_parked.SM_Mini2021_parked',
            'vehicle.mini.cooperst': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/Mini2021/SM_Mini2021_parked.SM_Mini2021_parked',  # Alternative naming
            'vehicle.mini.cooperst_2021': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/Mini2021/SM_Mini2021_parked.SM_Mini2021_parked',  # Alternative naming
            'vehicle.nissan.patrol': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/NissanPatrol2021/SM_NissanPatrol2021_parked.SM_NissanPatrol2021_parked',
            'vehicle.nissan.patrol_2021': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/NissanPatrol2021/SM_NissanPatrol2021_parked.SM_NissanPatrol2021_parked',
            'vehicle.tesla.model3': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/TeslaM3/SM_TeslaM3_parked.SM_TeslaM3_parked',
            'vehicle.tesla.model3_2021': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/TeslaM3/SM_TeslaM3_parked.SM_TeslaM3_parked',
            'vehicle.volkswagen.t2': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/VolkswagenT2/SM_VolkswagenT2_2021_Parked.SM_VolkswagenT2_2021_Parked',
            'vehicle.volkswagen.t2_2021': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/VolkswagenT2/SM_VolkswagenT2_2021_Parked.SM_VolkswagenT2_2021_Parked',
            
            # ===== Individual vehicle folders with parked meshes =====
            'vehicle.audi.etron': '/Game/Carla/Static/Car/4Wheeled/AudiETron/SM_EtronParked.SM_EtronParked',
            'vehicle.audi.tt': '/Game/Carla/Static/Car/4Wheeled/AudiTT/SM_AudiTT.SM_AudiTT',
            'vehicle.audi.a2': '/Game/Carla/Static/Car/4Wheeled/AudiA2/SM_AudiA2.SM_AudiA2',
            'vehicle.bmw.grandtourer': '/Game/Carla/Static/Car/4Wheeled/BmwGranTourer/SM_BMWGrandTourer.SM_BMWGrandTourer',
            'vehicle.bmw.isetta': '/Game/Carla/Static/Car/4Wheeled/BmwIsetta/SM_BMWIsetta.SM_BMWIsetta',
            'vehicle.chevrolet.impala': '/Game/Carla/Static/Car/4Wheeled/Chevrolet/SM_ChevroletImpala.SM_ChevroletImpala',
            'vehicle.citroen.c3': '/Game/Carla/Static/Car/4Wheeled/Citroen/SM_Citroen_C3.SM_Citroen_C3',
            'vehicle.tesla.cybertruck': '/Game/Carla/Static/Car/4Wheeled/Cybertruck/SM_Cybertruck.SM_Cybertruck',
            'vehicle.cybertruck': '/Game/Carla/Static/Car/4Wheeled/Cybertruck/SM_Cybertruck.SM_Cybertruck',  # Alternative naming
            'vehicle.dodge.charger_2020': '/Game/Carla/Static/Car/4Wheeled/DodgeCharger2020/SM_Charger_parked.SM_Charger_parked',
            'vehicle.dodge.charger_police_2020': '/Game/Carla/Static/Car/4Wheeled/DodgeCharger2020/ChargerCop/SM_ChargerCopParked.SM_ChargerCopParked',
            'vehicle.ford.mustang': '/Game/Carla/Static/Car/4Wheeled/Mustang/SM_Mustang_prop.SM_Mustang_prop',
            'vehicle.jeep.wrangler_rubicon': '/Game/Carla/Static/Car/4Wheeled/Jeep/SM_JeepWranglerRubicon.SM_JeepWranglerRubicon',
            'vehicle.seat.leon': '/Game/Carla/Static/Car/4Wheeled/Leon/SM_SeatLeon.SM_SeatLeon',
            'vehicle.lincoln.mkz_2020': '/Game/Carla/Static/Car/4Wheeled/LincolnMKZ2020/SM_Lincoln2020Parked.SM_Lincoln2020Parked',
            'vehicle.mercedes.coupe': '/Game/Carla/Static/Car/4Wheeled/MercedesCCC/SM_mercedescccParked.SM_mercedescccParked',
            'vehicle.mini.cooper_s': '/Game/Carla/Static/Car/4Wheeled/Mini2021/SM_Mini2021Parked.SM_Mini2021Parked',
            'vehicle.mini.cooper_s_2021': '/Game/Carla/Static/Car/4Wheeled/Mini2021/SM_Mini2021Parked.SM_Mini2021Parked',
            'vehicle.mini.cooperst_2021': '/Game/Carla/Static/Car/4Wheeled/Mini2021/SM_Mini2021Parked.SM_Mini2021Parked',  # Alternative naming
            'vehicle.nissan.micra': '/Game/Carla/Static/Car/4Wheeled/Nissan_Micra/SM_NissanMicra.SM_NissanMicra',
            'vehicle.nissan.patrol_2021': '/Game/Carla/Static/Car/4Wheeled/NissanPatrol2021/SM_Patrol2021Parked.SM_Patrol2021Parked',
            'vehicle.tesla.model3': '/Game/Carla/Static/Car/4Wheeled/Tesla/SM_TeslaM3_v2.SM_TeslaM3_v2',
            'vehicle.toyota.prius': '/Game/Carla/Static/Car/4Wheeled/Toyota_Prius/SMC_ToyotaPrius.SMC_ToyotaPrius',
            'vehicle.volkswagen.beetle': '/Game/Carla/Static/Car/4Wheeled/Beetle/SM_VolkswagenBeetle.SM_VolkswagenBeetle',
            'vehicle.volkswagen.t2': '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/VolkswagenT2/SM_VolkswagenT2_2021_Parked.SM_VolkswagenT2_2021_Parked',
            'vehicle.tazzari.zero': '/Game/Carla/Static/Car/4Wheeled/Tazzari/SM_Tazzari.SM_Tazzari',
            'vehicle.roameo': '/Game/Carla/Static/Car/4Wheeled/ROAMEO/ROAMEO.ROAMEO',
            
            # Note: Vehicles without static meshes will fall back to vehicle blueprints:
            # - vehicle.carlamotors.carlacola
            # - vehicle.micro.microlino
            # - vehicle.mercedes.sprinter
            # - vehicle.ford.ambulance
            # These will use vehicle blueprints with physics enabled
        }
        
        if vehicle_bp_id in VEHICLE_MESH_MAPPING:
            return VEHICLE_MESH_MAPPING[vehicle_bp_id]
        
        # Try to construct path from vehicle ID (may not work for all vehicles)
        # Pattern: vehicle.make.model -> /Game/Carla/Static/Car/4Wheeled/ParkedVehicles/MakeModel/SM_MakeModelParked.SM_MakeModelParked
        parts = vehicle_bp_id.split('.')
        if len(parts) >= 3:
            make = parts[1].capitalize()
            model = parts[2].capitalize()
            
            # Try common patterns (unlikely to work without actual paths)
            # This is just a placeholder - you should add actual mappings above
            pass
        
        # Return None if no path found - will fall back to vehicle blueprint
        return None
    
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
    
    def save_spawn_data(self, spawn_data, filename):
        """
        Save spawn data to a JSON file.
        
        Args:
            spawn_data: List of dicts with spawn information
            filename: Output filename
        """
        try:
            with open(filename, 'w') as f:
                json.dump(spawn_data, f, indent=2)
            print(f"\nSpawn data saved to: {filename}")
        except Exception as e:
            print(f"Error saving spawn data: {e}")
    
    def load_spawn_data(self, filename):
        """
        Load spawn data from a JSON file.
        
        Args:
            filename: Input filename
            
        Returns:
            list: List of dicts with spawn information, or None if error
        """
        try:
            with open(filename, 'r') as f:
                spawn_data = json.load(f)
            print(f"Loaded spawn data from: {filename}")
            print(f"  Found {len(spawn_data)} vehicle spawns")
            return spawn_data
        except FileNotFoundError:
            print(f"Error: File not found: {filename}")
            return None
        except Exception as e:
            print(f"Error loading spawn data: {e}")
            return None
    
    def spawn_parked_vehicle(self, vehicle_bp, transform, vehicle_bp_id=None):
        """
        Spawn a parked vehicle as either a static prop (better performance) or vehicle blueprint.
        
        Args:
            vehicle_bp: Vehicle blueprint (used for fallback or when use_static_props=False)
            transform: Spawn transform
            vehicle_bp_id: Vehicle blueprint ID (for mesh path lookup)
            
        Returns:
            carla.Actor: Spawned actor (static prop or vehicle), or None if failed
        """
        if vehicle_bp_id is None:
            vehicle_bp_id = vehicle_bp.id
        
        # Try to use static prop if enabled
        if self.use_static_props and self.static_prop_bp is not None:
            mesh_path = self.get_vehicle_mesh_path(vehicle_bp_id)
            
            if mesh_path:
                try:
                    # Set attributes on the blueprint (CARLA creates a new actor description on spawn)
                    # Note: We set attributes before each spawn to ensure correct mesh_path
                    prop_bp = self.static_prop_bp
                    prop_bp.set_attribute('mesh_path', mesh_path)
                    prop_bp.set_attribute('mass', str(self.prop_mass))  # Enable physics with mass
                    
                    # Spawn static prop (physics automatically enabled when mass > 0)
                    prop = self.world.spawn_actor(prop_bp, transform)
                    
                    print(f"Spawned static prop: {mesh_path} (mass={self.prop_mass}kg) at "
                          f"({transform.location.x:.2f}, {transform.location.y:.2f}, {transform.location.z:.2f})")
                    return prop
                except Exception as e:
                    print(f"Warning: Failed to spawn static prop for {vehicle_bp_id}: {e}")
                    print(f"  Falling back to vehicle blueprint...")
                    # Fall through to vehicle blueprint spawn
        
        # Fallback: Use vehicle blueprint (original method)
        try:
            vehicle = self.world.spawn_actor(vehicle_bp, transform)
            vehicle.set_simulate_physics(True)
            vehicle.set_enable_gravity(True)
            vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
            
            control = carla.VehicleControl()
            control.brake = 0.5
            control.hand_brake = False
            control.throttle = 0.0
            control.steer = 0.0
            vehicle.apply_control(control)
            
            print(f"Spawned vehicle blueprint: {vehicle_bp.id} at "
                  f"({transform.location.x:.2f}, {transform.location.y:.2f}, {transform.location.z:.2f})")
            return vehicle
        except Exception as e:
            print(f"Failed to spawn vehicle: {e}")
            return None
    
    def spawn_vehicles_along_line(self, parking_line_data, vehicle_blueprints, loaded_spawns=None, parking_line_index=None):
        """
        Spawn vehicles along a single parking middle line.
        
        Args:
            parking_line_data: [start_loc, end_loc, side_type, num_vehicles, min_spacing, exclude_vehicles]
                exclude_vehicles is optional and can be a list of vehicle keywords to exclude
            vehicle_blueprints: List of vehicle blueprints to choose from
            loaded_spawns: Optional list of pre-loaded spawn data (from file) for THIS parking line only
            parking_line_index: Index of this parking line (for tracking in save data)
            
        Returns:
            tuple: (list of spawned vehicle actors, list of spawn data dicts)
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
            return [], []
        
        spawned_vehicles = []
        spawn_data_list = []
        
        # If loaded spawns provided, use them; otherwise generate new positions
        if loaded_spawns:
            # Use loaded spawn data
            for spawn_info in loaded_spawns:
                try:
                    # Create transform from loaded data
                    location = carla.Location(
                        x=spawn_info['location']['x'],
                        y=spawn_info['location']['y'],
                        z=spawn_info['location']['z']
                    )
                    rotation = carla.Rotation(
                        yaw=spawn_info['rotation']['yaw'],
                        pitch=spawn_info['rotation']['pitch'],
                        roll=spawn_info['rotation']['roll']
                    )
                    transform = carla.Transform(location, rotation)
                    
                    # Find vehicle blueprint by ID
                    vehicle_bp_id = spawn_info.get('vehicle_type', spawn_info.get('mesh_path', ''))
                    vehicle_bp = None
                    for bp in line_vehicle_blueprints:
                        if bp.id == vehicle_bp_id:
                            vehicle_bp = bp
                            break
                    
                    if vehicle_bp is None:
                        print(f"Warning: Vehicle type '{vehicle_bp_id}' not found, skipping...")
                        continue
                    
                    # Spawn using helper method (handles static prop vs vehicle)
                    actor = self.spawn_parked_vehicle(vehicle_bp, transform, vehicle_bp_id)
                    
                    if actor is not None:
                        spawned_vehicles.append(actor)
                        spawn_data_list.append(spawn_info)  # Keep same data
                    
                except Exception as e:
                    print(f"Failed to spawn vehicle from loaded data: {e}")
        else:
            # Generate spawn positions (original behavior)
            spawn_transforms = self.generate_spawn_positions(
                start_loc, end_loc, side_type, num_vehicles, min_spacing
            )
            
            for transform, side in spawn_transforms:
                # Randomly select a vehicle blueprint from filtered list
                vehicle_bp = random.choice(line_vehicle_blueprints)
                
                # Spawn using helper method (handles static prop vs vehicle)
                actor = self.spawn_parked_vehicle(vehicle_bp, transform)
                
                if actor is not None:
                    spawned_vehicles.append(actor)
                    
                    # Save spawn data (with parking_line_index for tracking)
                    spawn_data = {
                        'location': {
                            'x': transform.location.x,
                            'y': transform.location.y,
                            'z': transform.location.z
                        },
                        'rotation': {
                            'yaw': transform.rotation.yaw,
                            'pitch': transform.rotation.pitch,
                            'roll': transform.rotation.roll
                        },
                        'vehicle_type': vehicle_bp.id,
                        'side': side,
                        'parking_line_index': parking_line_index
                    }
                    spawn_data_list.append(spawn_data)
                    
                    print(f"  on {side} side (yaw: {transform.rotation.yaw:.1f}°)")
        
        return spawned_vehicles, spawn_data_list
    
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
    
    def spawn_carpark(self, parking_lines, vehicle_filter='vehicle.*', load_file=None):
        """
        Spawn vehicles for all parking lines defined in the input array.
        
        Args:
            parking_lines: List of parking line data arrays
            vehicle_filter: Blueprint filter string for vehicles
            load_file: Optional filename to load spawn data from
            
        Returns:
            tuple: (list of all spawned vehicle actors, list of all spawn data)
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
        
        # Load spawn data from file if provided and organize by parking line index
        loaded_spawns_by_line = None
        if load_file:
            all_loaded_spawns = self.load_spawn_data(load_file)
            if all_loaded_spawns is None:
                print("Failed to load spawn data. Falling back to random generation.")
                loaded_spawns_by_line = None
            else:
                # Group spawns by parking_line_index
                loaded_spawns_by_line = {}
                for spawn in all_loaded_spawns:
                    line_idx = spawn.get('parking_line_index', None)
                    if line_idx is not None:
                        if line_idx not in loaded_spawns_by_line:
                            loaded_spawns_by_line[line_idx] = []
                        loaded_spawns_by_line[line_idx].append(spawn)
                    else:
                        # Legacy format: spawns without parking_line_index
                        # Assign them sequentially (not ideal but handles old files)
                        print("Warning: Found spawns without parking_line_index. Assigning sequentially.")
                        # We'll handle this case by using spawns in order
                        if 'unassigned' not in loaded_spawns_by_line:
                            loaded_spawns_by_line['unassigned'] = []
                        loaded_spawns_by_line['unassigned'].append(spawn)
        
        print(f"Processing {len(parking_lines)} parking lines...\n")
        
        all_spawn_data = []
        unassigned_spawn_index = 0
        
        # Spawn vehicles for each parking line
        for i, parking_line in enumerate(parking_lines):
            print(f"Processing parking line {i+1}/{len(parking_lines)}...")
            
            # Show exclude filters if present
            if len(parking_line) >= 6 and parking_line[5]:
                exclude_list = parking_line[5]
                print(f"  Excluding vehicles: {exclude_list}")
            
            # Get spawns for this specific parking line
            line_loaded_spawns = None
            if loaded_spawns_by_line:
                # Try to get spawns for this line index
                if i in loaded_spawns_by_line:
                    line_loaded_spawns = loaded_spawns_by_line[i]
                    print(f"  Using {len(line_loaded_spawns)} pre-saved spawns for this line")
                elif 'unassigned' in loaded_spawns_by_line and loaded_spawns_by_line['unassigned']:
                    # Legacy format: use unassigned spawns sequentially
                    # This is a fallback for old file formats
                    remaining_unassigned = loaded_spawns_by_line['unassigned'][unassigned_spawn_index:]
                    if remaining_unassigned:
                        line_loaded_spawns = remaining_unassigned
                        unassigned_spawn_index += len(remaining_unassigned)
                        print(f"  Using {len(line_loaded_spawns)} unassigned spawns for this line")
            
            vehicles, spawn_data = self.spawn_vehicles_along_line(
                parking_line, vehicle_blueprints, line_loaded_spawns, parking_line_index=i
            )
            self.vehicles.extend(vehicles)
            all_spawn_data.extend(spawn_data)
            print(f"  Spawned {len(vehicles)} vehicles\n")
        
        print(f"Total vehicles spawned: {len(self.vehicles)}\n")
        
        return self.vehicles, all_spawn_data
    
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
    argparser.add_argument(
        '-f', '--file',
        metavar='F',
        default=None,
        help='Load spawn data from JSON file. If not provided, spawns randomly and saves to carpark_spawns.json')
    argparser.add_argument(
        '-s', '--save',
        metavar='S',
        default='carpark_spawns.json',
        help='Filename to save spawn data (default: carpark_spawns.json)')
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
                "both",      # Side type parking_lines
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
        # By default, uses static.prop.mesh for better performance (no vehicle component overhead)
        # Falls back to vehicle blueprints if mesh paths not found
        spawner = CarparkVehicleSpawner(
            world=world,
            spawn_height=1.5,        # Spawn 1.5m above ground
            parking_offset=3.0,     # 3.0m offset from middle line
            use_static_props=True,   # Use static props (better performance)
            prop_mass=100.0          # Mass for static props (enables physics)
        )
        
        if spawner.use_static_props:
            print("Using static.prop.mesh for parked vehicles (better performance)")
            print("  - Physics enabled via mass attribute")
            print("  - Falls back to vehicle blueprints if mesh paths not found")
        else:
            print("Using vehicle blueprints (fallback mode)")
        
        # Spawn all vehicles
        vehicles, spawn_data = spawner.spawn_carpark(
            parking_lines=parking_lines,
            vehicle_filter='vehicle.*',  # Use all vehicle types
            load_file=args.file  # Load from file if provided
        )
        
        # Save spawn data if not loading from file
        if not args.file:
            spawner.save_spawn_data(spawn_data, args.save)
            print(f"Spawn data saved to: {args.save}")
        
        print(f"\nSuccessfully spawned {len(vehicles)} vehicles with physics enabled.")
        print("Vehicles will fall and settle naturally in CARLA's physics simulation.")
        print("Script exiting - vehicles remain in the world as physics-enabled objects.\n")
    
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

