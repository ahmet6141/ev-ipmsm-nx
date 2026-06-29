"""Runnable FOC / field-weakening / regen system simulation for the EV IPMSM.
Parameters DERIVED from motor_nx.em_design (documented below). This is the
'Kontrol/sistem' analysis -- it actually runs here (numpy/scipy/matplotlib)."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from motor_nx import em_design as emd
from motor_nx.params import MotorParams

p = MotorParams()
# ---- derive the dq machine model from the first-order design --------------- #
pp = p.rotor.pole_count // 2                      # pole pairs = 3
n_base, n_max = 3623.0, 18000.0
Vdc = 400.0
Imax = 273.0 * np.sqrt(2)                          # peak current amplitude (A)
# lambda_m from back-EMF constant 78.1 V/1000rpm (LL rms):
ke_ll_rms_per_krpm = 78.1
w_e_1krpm = (1000.0/60.0)*pp*2*np.pi               # elec rad/s at 1000 rpm
E_peak_phase = ke_ll_rms_per_krpm*np.sqrt(2)/np.sqrt(3)   # peak phase EMF at 1000rpm
lam_m = E_peak_phase / w_e_1krpm                   # Wb (peak flux linkage)
# Rs from continuous copper loss 1.78 kW at 126 A rms, 3 phases:
Rs = 1780.0/(3*126.0**2)
# Ld from matching characteristic current to ~Imax (deep field weakening, CPSR~5);
# Lq = saliency * Ld (em_design saliency 1.25):
Ld = lam_m/Imax
Lq = 1.25*Ld
Vmax = Vdc/np.sqrt(3)                               # SVPWM phase-voltage limit (peak)
print("=== derived dq model ===")
print(f"pole pairs={pp}  lambda_m={lam_m*1e3:.1f} mWb  Rs={Rs*1e3:.1f} mOhm")
print(f"Ld={Ld*1e6:.0f} uH  Lq={Lq*1e6:.0f} uH  saliency={Lq/Ld:.2f}")
print(f"Imax(amp)={Imax:.0f} A  Vmax(phase pk)={Vmax:.0f} V  Vdc={Vdc:.0f} V")

def torque(id_, iq_):
    return 1.5*pp*(lam_m*iq_ + (Ld-Lq)*id_*iq_)

# ---- torque-speed envelope: max torque per speed under current+voltage limits #
speeds = np.linspace(100, n_max, 120)
n_id, n_iq = 60, 60
ids = np.linspace(-Imax, 0, n_id)
T_env, P_env = [], []
for n in speeds:
    we = (n/60.0)*pp*2*np.pi
    best_T = 0.0
    for idv in ids:
        iq_lim = np.sqrt(max(0.0, Imax**2 - idv**2))
        iqs = np.linspace(0, iq_lim, n_iq)
        for iqv in iqs:
            vd = Rs*idv - we*Lq*iqv
            vq = Rs*iqv + we*(lam_m + Ld*idv)
            if vd*vd+vq*vq <= Vmax*Vmax:
                T = torque(idv, iqv)
                if T > best_T:
                    best_T = T
    T_env.append(best_T)
    P_env.append(best_T*we/pp)        # mech power = T*omega_mech = T*we/pp
T_env, P_env = np.array(T_env), np.array(P_env)
i_pk = int(np.argmax(T_env)); i_pmax = int(np.argmax(P_env))
print("\n=== torque-speed envelope (MTPA + field weakening) ===")
print(f"peak torque (low speed)  : {T_env.max():.0f} Nm   (design peak 440 Nm)")
print(f"peak power               : {P_env.max()/1e3:.0f} kW  @ {speeds[i_pmax]:.0f} rpm  (design 167 kW)")
print(f"torque @ {speeds[-1]:.0f} rpm  : {T_env[-1]:.0f} Nm,  power {P_env[-1]/1e3:.0f} kW")

# ---- FOC d-axis/q-axis PI current-loop step response (closed loop) --------- #
# tune PI to a target current-loop bandwidth (e.g. 1 kHz) by pole placement
fbw = 1000.0; wbw = 2*np.pi*fbw
KpD, KiD = Ld*wbw, Rs*wbw
KpQ, KiQ = Lq*wbw, Rs*wbw
dt = 2e-6; tend = 3e-3; N = int(tend/dt)
t = np.arange(N)*dt
we = 0.0                          # locked-rotor (standard current-loop bandwidth test)
id_r, iq_r = -150.0, 250.0      # step references (field-weakening id, torque iq)
id_, iq_, intD, intQ = 0.0,0.0,0.0,0.0
ID,IQ = np.zeros(N),np.zeros(N)
for k in range(N):
    eD, eQ = id_r-id_, iq_r-iq_
    intD += eD*dt; intQ += eQ*dt
    vd = KpD*eD+KiD*intD - we*Lq*iq_      # PI + cross-decoupling
    vq = KpQ*eQ+KiQ*intQ + we*(lam_m+Ld*id_)
    vm = np.hypot(vd,vq)                   # voltage saturation (SVPWM circle)
    if vm>Vmax: vd*=Vmax/vm; vq*=Vmax/vm
    did = (vd - Rs*id_ + we*Lq*iq_)/Ld
    diq = (vq - Rs*iq_ - we*(lam_m+Ld*id_))/Lq
    id_+=did*dt; iq_+=diq*dt
    ID[k],IQ[k]=id_,iq_
# rise time (10-90%) of iq -- guard against never-reached (voltage-limited) case
print("\n=== FOC current loop (1 kHz target, locked rotor) ===")
print(f"KpD={KpD:.4f} KiD={KiD:.1f} | KpQ={KpQ:.4f} KiQ={KiQ:.1f}")
if IQ.max() >= 0.9*iq_r:
    i10 = int(np.argmax(IQ >= 0.1*iq_r)); i90 = int(np.argmax(IQ >= 0.9*iq_r))
    print(f"iq 10-90% rise time      : {(t[i90]-t[i10])*1e6:.0f} us  (target ~{1e6*0.35/fbw:.0f} us)")
else:
    print(f"iq did not reach 90%% of {iq_r:.0f} A (voltage-limited); max {IQ.max():.0f} A")

# ---- regen braking operating point ----------------------------------------- #
n_regen = 9000.0; we_r=(n_regen/60.0)*pp*2*np.pi
iq_regen = -200.0; id_regen=-120.0
T_regen = torque(id_regen, iq_regen)
P_regen = T_regen*we_r/pp
batt_limit_kw = 70.0
P_eff = max(P_regen/1e3, -batt_limit_kw)   # clamp to battery charge limit
print("\n=== regenerative braking @ 9000 rpm ===")
print(f"braking torque           : {T_regen:.0f} Nm (negative = generating)")
print(f"recovered power          : {P_regen/1e3:.0f} kW -> clamped to battery {batt_limit_kw} kW -> {P_eff:.0f} kW")

# ---- plots ------------------------------------------------------------------ #
fig,ax=plt.subplots(1,2,figsize=(12,4.5))
ax[0].plot(speeds,T_env,'b',label='Torque [Nm]'); ax[0].set_xlabel('rpm'); ax[0].set_ylabel('Torque [Nm]',color='b')
a2=ax[0].twinx(); a2.plot(speeds,P_env/1e3,'r',label='Power [kW]'); a2.set_ylabel('Power [kW]',color='r')
ax[0].axvline(n_base,ls='--',c='g',alpha=.6); ax[0].set_title('Torque-Speed Envelope (MTPA + Field Weakening)')
ax[1].plot(t*1e3,IQ,label='iq'); ax[1].plot(t*1e3,ID,label='id')
ax[1].axhline(iq_r,ls='--',c='gray'); ax[1].axhline(id_r,ls='--',c='gray')
ax[1].set_xlabel('t [ms]'); ax[1].set_ylabel('current [A]'); ax[1].set_title('FOC dq Current-Loop Step'); ax[1].legend()
plt.tight_layout(); plt.savefig('analysis_runs/control_results.png',dpi=110)
print("\nsaved plot -> analysis_runs/control_results.png")
