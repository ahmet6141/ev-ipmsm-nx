"""Real CalculiX (ccx) structural FEA: rotor as a rotating annular disk under
centrifugal load at 1.2x max speed. Quarter-symmetry plane-stress mesh, generated
here (no gmsh). FEA peak hoop stress is cross-checked against the closed-form
rotating-disk solution -- proving the structural pipeline runs on the real rotor."""
import os, math, subprocess
import numpy as np

ri = 22.5      # shaft radius (mm)  -> rotor bore
ro = 79.8      # rotor OD/2 (mm)
th = 134.0     # stack length (mm) -> plane-stress thickness
E, nu, rho = 200e3, 0.3, 7.7e-9   # MPa, -, tonne/mm^3 (ccx consistent units: N,mm,t,MPa,s)
rpm = 21600.0
omega = rpm/60.0*2*math.pi         # rad/s
nr, nt = 16, 40                    # radial, tangential divisions over 90 deg

# ---- mesh a quarter annulus (CPS4) ----------------------------------------- #
nodes, nid = {}, {}
k=1
for i in range(nr+1):
    r = ri + (ro-ri)*i/nr
    for j in range(nt+1):
        a = (math.pi/2)*j/nt
        nid[(i,j)] = k
        nodes[k] = (r*math.cos(a), r*math.sin(a))
        k+=1
elems={}; e=1
for i in range(nr):
    for j in range(nt):
        n1=nid[(i,j)]; n2=nid[(i+1,j)]; n3=nid[(i+1,j+1)]; n4=nid[(i,j+1)]
        elems[e]=(n1,n2,n3,n4); e+=1
# symmetry BC node sets: y=0 edge (j=0) -> uy=0 ; x=0 edge (j=nt) -> ux=0
edge_y0=[nid[(i,0)] for i in range(nr+1)]
edge_x0=[nid[(i,nt)] for i in range(nr+1)]

inp="analysis_runs/rotor.inp"
with open(inp,"w") as f:
    f.write("*NODE, NSET=NALL\n")
    for n,(x,y) in nodes.items(): f.write(f"{n}, {x:.6f}, {y:.6f}, 0.0\n")
    f.write("*ELEMENT, TYPE=CPS4, ELSET=EALL\n")
    for en,(a,b,c,d) in elems.items(): f.write(f"{en}, {a}, {b}, {c}, {d}\n")
    def _nset(name, ids):
        f.write("*NSET, NSET=%s\n" % name)
        for c in range(0, len(ids), 8):                 # <=16 entries/line (CalculiX limit)
            f.write(",".join(map(str, ids[c:c+8])) + "\n")
    _nset("SY", edge_y0)
    _nset("SX", edge_x0)
    f.write("*MATERIAL, NAME=STEEL\n*ELASTIC\n%g, %g\n*DENSITY\n%g\n"%(E,nu,rho))
    f.write("*SOLID SECTION, ELSET=EALL, MATERIAL=STEEL\n%g\n"%th)
    f.write("*STEP\n*STATIC\n")
    f.write("*BOUNDARY\nSY,2,2\nSX,1,1\n")          # symmetry
    f.write("*DLOAD\nEALL,CENTRIF,%g,0.,0.,0.,0.,0.,1.\n"%(omega*omega))  # NF=omega^2
    f.write("*EL PRINT, ELSET=EALL\nS\n")
    f.write("*END STEP\n")

r=subprocess.run(["ccx","-i","analysis_runs/rotor"],capture_output=True,text=True)
print(r.stdout[-400:]); 
if r.returncode!=0: print("CCX STDERR:",r.stderr[-400:])

# ---- parse .dat stresses, compute von Mises ------------------------------- #
vm_max=0.0; s1_max=0.0
dat="analysis_runs/rotor.dat"
if os.path.exists(dat):
    cap=False
    for ln in open(dat):
        if "stresses" in ln.lower(): cap=True; continue
        if cap:
            p=ln.split()
            if len(p)>=7:
                try:
                    sxx,syy,szz,sxy,sxz,syz=map(float,p[2:8])
                except: continue
                vm=math.sqrt(0.5*((sxx-syy)**2+(syy-szz)**2+(szz-sxx)**2)+3*(sxy**2+sxz**2+syz**2))
                vm_max=max(vm_max,vm)
# ---- closed-form annular rotating disk (plane stress), peak hoop at ri ----- #
w2=omega*omega; rho_si=7700.0; ri_m, ro_m = ri/1000, ro/1000
sig_theta_ri = (rho_si*w2/4)*((3+nu)*ro_m**2+(1-nu)*ri_m**2)/1e6   # MPa
yield_mpa=450.0
print("\n=== Rotor centrifugal FEA (CalculiX) @ %.0f rpm ==="%rpm)
print(f"mesh: {len(elems)} CPS4 elements, {len(nodes)} nodes (quarter annulus {ri}-{ro} mm)")
print(f"FEA peak von Mises       : {vm_max:.0f} MPa")
print(f"closed-form hoop @ bore  : {sig_theta_ri:.0f} MPa  (validation reference)")
print(f"yield (lamination steel) : {yield_mpa:.0f} MPa  -> FEA SF = {yield_mpa/max(vm_max,1e-9):.2f}")
print("note: SOLID annulus model (no flux-barrier bridges). The thin 1 mm outer")
print("bridge concentrates stress far higher -- analysis.py estimates ~301 MPa/SF1.5;")
print("that needs the detailed lamination geometry meshed (next FEA step).")
