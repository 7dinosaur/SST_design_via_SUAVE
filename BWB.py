# tut_Concorde.py
# 
# Created:  Aug 2014, SUAVE Team
# Modified: Jan 2017, T. MacDonald
#           Aug 2017, E. Botero

""" setup file for a mission with a concorde
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
assert SUAVE.__version__=='2.5.2', 'These tutorials only work with the SUAVE 2.5.2 release'
from SUAVE.Core import Units, Data
from SUAVE.Plots.Performance.Mission_Plots import *
from SUAVE.Methods.Propulsion.turbojet_sizing import turbojet_sizing
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import segment_properties
from SUAVE.Input_Output.OpenVSP import write

from copy import deepcopy

import pandas as pd
from wing_tool import wing_from_para

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():

    configs, analyses = full_setup()
    print(analyses.missions.base.segments.cruise.distance / Units.km)
    max_mass = configs.base.mass_properties.max_takeoff
    print("最大起飞质量：", max_mass)

    simple_sizing(configs)

    print("初始化...")
    configs.finalize()
    analyses.finalize()

    # weight analysis
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()      

    print("分析...")
    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()
    
    #-------------------------------------------------------------------------------------------------------
    # 初始分析
    #-------------------------------------------------------------------------------------------------------
    last_segment = results.segments.values()[-1]
    final_mass     = last_segment.conditions.weights.total_mass[-1, 0]
    range_list = []
    for segment in results.segments.values():
        range_list.append(segment.conditions.frames.inertial.aircraft_range[-1, 0])
    print("最终质量：", final_mass, "燃油消耗：", max_mass - final_mass)
    print("分段航程：", range_list)
    # other_range = (range_list[2] + range_list[4] - range_list[3] + range_list[-1] - range_list[-4]) / 1000
    # range_13 = range_list[2] / 1000
    # print("爬升下降所用航程：", other_range)
    # target_range = 10388.825148624662
    # land_range = 1315
    # # land_range = range_13
    # wait_cruise_range = (land_range - range_13)
    # print("马赫截止巡航航程: ", wait_cruise_range / Units.km)
    # true_cruise_range = (target_range - other_range - wait_cruise_range)
    # analyses.missions.base.segments.cruise_0.distance = wait_cruise_range * Units.km
    # analyses.missions.base.segments.cruise.distance = true_cruise_range * Units.km

    # print("修正航程后重新分析...")
    # # mission analysis
    # mission = analyses.missions.base
    # print("确认新巡航段: ", mission.segments.cruise.distance)
    # print("确认马赫截止巡航段: ", mission.segments.cruise_0.distance)
    # results = mission.evaluate()
    
    # ## 确定飞行任务后第一轮分析得到燃油消耗
    # last_segment = results.segments.values()[-1]
    # final_mass     = last_segment.conditions.weights.total_mass[-1, 0]
    # range_list = []
    # for segment in results.segments.values():
    #     range_list.append(segment.conditions.frames.inertial.aircraft_range[-1, 0])
    # print("最终质量：", final_mass, "燃油消耗：", max_mass - final_mass)
    # print("分段航程：", range_list)
    # other_range = (range_list[4] + range_list[-1] - range_list[-4]) / 1000
    # print("爬升下降所用航程：", other_range)

    # mass_pass     = configs.base.mass_properties.cargo
    # empty_ratio   = 0.425
    # count = 0
    # maxiter = 10
    # while True:
    #     count += 1
    #     if count > maxiter:
    #         break

    #     last_segment  = results.segments.values()[-1]
    #     takeoff_mass  = configs.base.mass_properties.takeoff
    #     final_mass    = last_segment.conditions.weights.total_mass[-1, 0]
    #     fuel_mass     = takeoff_mass - final_mass
    #     new_takeoff   = (1/(1-empty_ratio)) * (fuel_mass + mass_pass)
    #     configs.base.mass_properties.takeoff = new_takeoff
    #     print(final_mass, mass_pass, takeoff_mass)
    #     print(f"第{count}次重量迭代", "空重系数：", (final_mass - mass_pass)/takeoff_mass, "总重：", takeoff_mass)
    #     results = mission.evaluate()

    plot_mission(results)
    
    plt.show()

    return

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():

    # vehicle data
    vehicle  = vehicle_setup()
    write(vehicle, 'bwb_6.64_simple')
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    missions_analyses  = mission_setup(configs_analyses)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses
    
    return configs, analyses

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_Transport()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.SU2_Euler()

    aerodynamics.geometry = vehicle
    aerodynamics.process.compute.lift.inviscid.training_file = 'base_data.txt'
    
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    aerodynamics.settings.span_efficiency            = .8

    aerodynamics.process.compute.lift.inviscid.training.Mach               = np.array([.3, .6, .9, 1.2, 1.5, 1.8]) 
    aerodynamics.process.compute.lift.inviscid.training.angle_of_attack    = np.array([0., 3., 6., 9., 12., 15.]) * Units.deg
    
    analyses.append(aerodynamics)
    # ------------------------------------------------------------------
    #  Stability Analysis
    
    # Not yet available for this configuration

    # ------------------------------------------------------------------
    #  Energy
    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.networks #what is called throughout the mission (at every time step))
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    # done!
    return analyses    

def vehicle_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    
    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Concorde'    
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff     = 120000. * Units.kilogram   
    vehicle.mass_properties.operating_empty = 78700.  * Units.kilogram   
    vehicle.mass_properties.takeoff         = 120000. * Units.kilogram   
    vehicle.mass_properties.cargo           = 12800.   * Units.kilogram   
        
    # envelope properties
    vehicle.envelope.ultimate_load = 3.75
    vehicle.envelope.limit_load    = 2.5

    # basic parameters
    vehicle.reference_area               = 300.6  
    vehicle.passengers                   = 160
    vehicle.systems.control              = "fully powered" 
    vehicle.systems.accessories          = "long range"
    vehicle.maximum_cross_sectional_area = 13.9
    vehicle.total_length                 = 72.0 * Units.meters
    
    
    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------
    try:
        wing = wing_from_para("6.64_simple.csv")
    except Exception as e:
        print(f"❌ wing_from_para 未实现或出错：{e}")
        exit()
    vehicle.append_component(wing) 

    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------
    
    # fuselage = SUAVE.Components.Fuselages.Fuselage()
    # fuselage.tag = 'fuselage'
    # fuselage.seats_abreast         = 4
    # fuselage.seat_pitch            = 1     * Units.meter
    # fuselage.fineness.nose         = 4.3   * Units.meter   
    # fuselage.fineness.tail         = 6.4   * Units.meter   
    # fuselage.lengths.total         = 61.66 * Units.meter    
    # fuselage.width                 = 2.88  * Units.meter   
    # fuselage.heights.maximum       = 3.32  * Units.meter   
    # fuselage.heights.maximum       = 3.32  * Units.meter   
    # fuselage.heights.at_quarter_length          = 3.32 * Units.meter   
    # fuselage.heights.at_wing_root_quarter_chord = 3.32 * Units.meter   
    # fuselage.heights.at_three_quarters_length   = 3.32 * Units.meter   
    # fuselage.areas.wetted          = 447. * Units['meter**2'] 
    # fuselage.areas.front_projected = 11.9 * Units['meter**2'] 
    # fuselage.effective_diameter    = 3.1 * Units.meter    
    # fuselage.differential_pressure = 7.4e4 * Units.pascal    # Maximum differential pressure
    
    # # add to vehicle
    # vehicle.append_component(fuselage)
    
    # ------------------------------------------------------------------        
    # the nacelle 
    # ------------------------------------------------------------------  
    
    # nacelle                  = SUAVE.Components.Nacelles.Nacelle()
    # nacelle.diameter         = 1.3
    # nacelle.tag              = 'nacelle_L1'
    # nacelle.origin           = [[60, 2, 2]] 
    # nacelle.length           = 8.0 
    # nacelle.inlet_diameter   = 1.1 
    # nacelle.areas.wetted     = 30.
    # vehicle.append_component(nacelle)       

    # nacelle_2               = deepcopy(nacelle)
    # nacelle_2.tag           = 'nacelle_2'
    # nacelle_2.origin        = [[60, -2, 2]]
    # vehicle.append_component(nacelle_2)     

    # nacelle_3               = deepcopy(nacelle)
    # nacelle_3.tag           = 'nacelle_3'
    # nacelle_3.origin        = [[37.,-5.3,-1.3]]
    # vehicle.append_component(nacelle_3)   

    # nacelle_4              = deepcopy(nacelle)
    # nacelle_4.tag          = 'nacelle_4'
    # nacelle_4.origin       = [[37.,-6.,-1.3]]
    # vehicle.append_component(nacelle_4)       
        
         
    # ------------------------------------------------------------------
    #   Turbojet Network
    # ------------------------------------------------------------------    
    
    # instantiate the gas turbine network
    turbojet = SUAVE.Components.Energy.Networks.Turbojet_Super()
    turbojet.tag = 'turbojet'
    
    # setup
    turbojet.number_of_engines = 4.0
    turbojet.engine_length     = 12.0
    turbojet.origin            = [[37.,6.,-1.3],[37.,5.3,-1.3],[37.,-5.3,-1.3],[37.,-6.,-1.3]] # meters
    
    # working fluid
    turbojet.working_fluid = SUAVE.Attributes.Gases.Air()

    # ------------------------------------------------------------------
    #   Component 1 - Ram
    
    # to convert freestream static to stagnation quantities
    
    # instantiate
    ram = SUAVE.Components.Energy.Converters.Ram()
    ram.tag = 'ram'
    
    # add to the network
    turbojet.append(ram)

    # ------------------------------------------------------------------
    #  Component 2 - Inlet Nozzle
    
    # instantiate
    inlet_nozzle = SUAVE.Components.Energy.Converters.Compression_Nozzle()
    inlet_nozzle.tag = 'inlet_nozzle'
    
    # setup
    inlet_nozzle.polytropic_efficiency = 0.98
    inlet_nozzle.pressure_ratio        = 1.0
    
    # add to network
    turbojet.append(inlet_nozzle)
    
    # ------------------------------------------------------------------
    #  Component 3 - Low Pressure Compressor
    
    # instantiate 
    compressor = SUAVE.Components.Energy.Converters.Compressor()    
    compressor.tag = 'low_pressure_compressor'

    # setup
    compressor.polytropic_efficiency = 0.91
    compressor.pressure_ratio        = 3.1    
    
    # add to network
    turbojet.append(compressor)

    # ------------------------------------------------------------------
    #  Component 4 - High Pressure Compressor
    
    # instantiate
    compressor = SUAVE.Components.Energy.Converters.Compressor()    
    compressor.tag = 'high_pressure_compressor'
    
    # setup
    compressor.polytropic_efficiency = 0.91
    compressor.pressure_ratio        = 5.0  
    
    # add to network
    turbojet.append(compressor)

    # ------------------------------------------------------------------
    #  Component 5 - Low Pressure Turbine
    
    # instantiate
    turbine = SUAVE.Components.Energy.Converters.Turbine()   
    turbine.tag='low_pressure_turbine'
    
    # setup
    turbine.mechanical_efficiency = 0.99
    turbine.polytropic_efficiency = 0.93     
    
    # add to network
    turbojet.append(turbine)
    
    # ------------------------------------------------------------------
    #  Component 6 - High Pressure Turbine
    
    # instantiate
    turbine = SUAVE.Components.Energy.Converters.Turbine()   
    turbine.tag='high_pressure_turbine'

    # setup
    turbine.mechanical_efficiency = 0.99
    turbine.polytropic_efficiency = 0.93     
    
    # add to network
    turbojet.append(turbine)
      
    # ------------------------------------------------------------------
    #  Component 7 - Combustor
    
    # instantiate    
    combustor = SUAVE.Components.Energy.Converters.Combustor()   
    combustor.tag = 'combustor'
    
    # setup
    combustor.efficiency                = 0.99   
    combustor.turbine_inlet_temperature = 1450.
    combustor.pressure_ratio            = 1.0
    combustor.fuel_data                 = SUAVE.Attributes.Propellants.Jet_A()    
    
    # add to network
    turbojet.append(combustor)

    # ------------------------------------------------------------------
    #  Component 8 - Core Nozzle
    
    # instantiate
    nozzle = SUAVE.Components.Energy.Converters.Supersonic_Nozzle()   
    nozzle.tag = 'core_nozzle'
    
    # setup
    nozzle.polytropic_efficiency = 0.95
    nozzle.pressure_ratio        = 0.99    
    
    # add to network
    turbojet.append(nozzle)

    
    # ------------------------------------------------------------------
    #Component 9 : Thrust (to compute the thrust)
    thrust = SUAVE.Components.Energy.Processes.Thrust()       
    thrust.tag ='compute_thrust'
 
    # total design thrust (includes all the engines)
    thrust.total_design             = 4*140000. * Units.N #Newtons
 
    # Note: Sizing builds the propulsor. It does not actually set the size of the turbojet
    # design sizing conditions
    altitude      = 0.0*Units.ft
    mach_number   = 0.01
    isa_deviation = 0.
    
    # add to network
    turbojet.thrust = thrust

    #size the turbojet
    turbojet_sizing(turbojet,mach_number,altitude)   
    
    # add  gas turbine network gt_engine to the vehicle
    vehicle.append_component(turbojet)      
    
    
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------
    
    configs = SUAVE.Components.Configs.Config.Container()
    
    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)
    
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    
    # config = SUAVE.Components.Configs.Config(base_config)
    # config.tag = 'cruise'
    
    # configs.append(config)
    
    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------
    
    # config = SUAVE.Components.Configs.Config(base_config)
    # config.tag = 'takeoff'
    
    # config.V2_VS_ratio = 1.21
    # config.maximum_lift_coefficient = 2.
    
    # configs.append(config)
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    # config = SUAVE.Components.Configs.Config(base_config)
    # config.tag = 'landing'

    # config.Vref_VS_ratio = 1.23
    # config.maximum_lift_coefficient = 2.
    
    # configs.append(config)
    
    return configs

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,line_style='bo-'):

    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style)
    
    # Plot Aerodynamic Forces 
    plot_aerodynamic_forces(results, line_style)
    
    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results, line_style)
    
    # Drag Components
    plot_drag_components(results, line_style)
    
    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results, line_style)
    
    # Plot Velocities 
    plot_aircraft_velocities(results, line_style)      

    return

def simple_sizing(configs):

    base = configs.base
    base.pull_base()

    # zero fuel weight
    base.mass_properties.max_zero_fuel = 0.9 * base.mass_properties.max_takeoff 

    # wing areas
    for wing in base.wings:
        wing.areas.wetted   = 2.0 * wing.areas.reference
        wing.areas.exposed  = 0.8 * wing.areas.wetted
        wing.areas.affected = 0.6 * wing.areas.wetted
        
    # fuselage seats
    # base.fuselages['fuselage'].number_coach_seats = base.passengers

    # diff the new data
    base.store_diff()


    # done!
    return

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses):
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'
    
    #airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    
    mission.airport = airport    
    
    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments
    
    # base segment
    base_segment = Segments.Segment()
    
    
    # ------------------------------------------------------------------
    #   First Climb Segment
    # ------------------------------------------------------------------
    
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1"
    
    segment.analyses.extend( analyses.base )
    
    # ones_row = segment.state.ones_row
    # segment.state.unknowns.body_angle = ones_row(1) * 7. * Units.deg   
    
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 3.0   * Units.km
    segment.air_speed      = 128.6 * Units['m/s']
    segment.climb_rate     = 12.0 * Units['m/s']
    
    # add to misison
    mission.append_segment(segment)
    
    
    # ------------------------------------------------------------------
    #   Second Climb Segment
    # ------------------------------------------------------------------    
    
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2"
    
    segment.analyses.extend( analyses.base )
    
    segment.altitude_end   = 11.0   * Units.km
    segment.air_speed      = 174.91  * Units['m/s']
    segment.climb_rate     = 4.0  * Units['m/s']
    
    # add to mission
    mission.append_segment(segment)
    
    
    # ------------------------------------------------------------------
    #   Third Climb Segment: linear Mach
    # ------------------------------------------------------------------    
    
    segment = Segments.Climb.Constant_Mach_Constant_Rate(base_segment)
    segment.tag = "climb_3"
    
    segment.analyses.extend( analyses.base )
    
    segment.altitude_end = 13.0   * Units.km
    # segment.mach_start   = 0.78
    # segment.mach_end     = 1.3
    segment.mach_number  = 1.3
    segment.climb_rate   = 6.0  * Units['m/s']
    
    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   马赫截止飞出陆地区域
    # ------------------------------------------------------------------  

    # segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    # segment.tag = "cruise_0"
    
    # segment.analyses.extend( analyses.base )
    
    # segment.mach       = 1.3
    # segment.distance   = 1000.0 * Units.km
        
    # mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Fourth Climb Segment: linear Mach
    # ------------------------------------------------------------------    
    
    segment = Segments.Climb.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "climb_4"
    
    segment.analyses.extend( analyses.base )
    
    segment.altitude_end = 18.0   * Units.km
    segment.mach_start   = 1.3
    segment.mach_end     = 1.8
    segment.climb_rate   = 6.0  * Units['m/s']
    
    # add to mission
    mission.append_segment(segment)
    

    # ------------------------------------------------------------------
    #   Fourth Climb Segment
    # ------------------------------------------------------------------    

    # segment = Segments.Climb.Constant_Mach_Constant_Rate(base_segment)
    # segment.tag = "climb_5"
    
    # segment.analyses.extend( analyses.base )
    
    # segment.altitude_end = 18.288   * Units.km
    # segment.mach_number  = 2.02
    # segment.climb_rate   = 0.65  * Units['m/s']
    
    # # add to mission
    # mission.append_segment(segment)
    
    # ------------------------------------------------------------------    
    #   Cruise Segment
    # ------------------------------------------------------------------    
    
    segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "cruise"
    
    segment.analyses.extend( analyses.base )
    
    segment.mach       = 1.8
    segment.distance   = 3844.0 * Units.km
        
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------    
    #   First Descent Segment
    # ------------------------------------------------------------------    
    
    segment = Segments.Descent.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "descent_1"
    
    segment.analyses.extend( analyses.base )
    
    segment.altitude_end = 11.0   * Units.km
    segment.mach_start   = 1.8
    segment.mach_end     = 0.95
    segment.descent_rate = 3.5   * Units['m/s']
    
    # add to mission
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------    
    #   Second Descent Segment
    # ------------------------------------------------------------------    
    
    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_2"
    
    segment.analyses.extend( analyses.base )
    
    segment.altitude_end = 3.0   * Units.km
    segment.air_speed    = 144.0 * Units['m/s']
    segment.descent_rate = 4.5   * Units['m/s']
    
    # add to mission
    mission.append_segment(segment)    
    
    # ------------------------------------------------------------------    
    #   Third Descent Segment
    # ------------------------------------------------------------------    

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_3"

    segment.analyses.extend( analyses.base )
    
    segment.altitude_end = 0.0   * Units.km
    segment.air_speed    = 130.0 * Units['m/s']
    segment.descent_rate = 2.5   * Units['m/s']

    # append to mission
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------    
    #   Mission definition complete    
    # ------------------------------------------------------------------
    missions = SUAVE.Analyses.Mission.Mission.Container()
    missions.base = mission

    return missions

if __name__ == '__main__': 
    
    main()