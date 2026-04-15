import subprocess

from pyxdsm.XDSM import XDSM, OPT, SOLVER, FUNC, LEFT

# Change `use_sfmath` to False to use computer modern
x = XDSM(use_sfmath=True)

x.add_system("design_system", OPT, r"\text{Overall Design}")
x.add_system("route_opt", OPT, r"\text{Route Optimizer}")
x.add_system("Range", FUNC, "Range")
x.add_system("land", FUNC, r"land \_ mask")
x.add_system("aircraft_opt", OPT, r"\text{Aircraft Optimizer}")
x.add_system("aerodynamic", FUNC, "aerodynamic")
x.add_system("SU2", SOLVER, r"SU2\_Euler")
x.add_system("CD_vis", FUNC, "C_{D,vis}")
x.add_system("SUAVE", SOLVER, r"\text{SUAVE}")
x.add_system("mission", FUNC, "mission")
x.add_system("weight", FUNC, "weight")

x.connect("design_system", "route_opt", "p_0,p_1")
x.connect("design_system", "aircraft_opt", "MTOW")
x.connect("route_opt", "Range", r"\Gamma")
x.connect("route_opt", "land", r"\Gamma")
x.connect("Range", "route_opt", "f")
x.connect("land", "route_opt", "c")
x.connect("route_opt", "aircraft_opt", r"\Gamma*")
x.connect("aircraft_opt", "SUAVE", r"MTOW, \Gamma*")
x.connect("aircraft_opt", "aerodynamic", r"x_0,x_1\dots x_N")
x.connect("aerodynamic", "SU2", r"x_0,x_1\dots x_N")
x.connect("aerodynamic", "CD_vis", r"x_0,x_1\dots x_N")
x.connect("SU2", "aerodynamic", "C_L, C_{D,inv}")
x.connect("CD_vis", "aerodynamic", "C_{D,vis}")
x.connect("aerodynamic", "SUAVE", r"C_L, C_D")
x.connect("SUAVE", "mission", r"MTOW, \Gamma*")
x.connect("aerodynamic", "mission", "C_L, C_D")
x.connect("mission", "weight", "D(t)")
x.connect("weight", "mission", "W(t)")
x.connect("mission", "aircraft_opt", "W_{fuel}")

x.add_input("design_system", "p_0, p_1, MTOW^{(0)}")
x.add_input("aircraft_opt", r"x_0,x_1\dots x_N")

x.add_output("route_opt", r"\Gamma*", side=LEFT)
x.add_output("aircraft_opt", "MTOW*, x*", side=LEFT)

filename = "mdf"
x.write(filename, build=True)
# command = ["xelatex", f"{filename}.tex"]
# print(command)
# subprocess.run(command, check=True, shell=True)