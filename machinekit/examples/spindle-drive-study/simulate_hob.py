#!/usr/bin/env python3
"""Local motor-driven hob master and assumed table servo; study, not HB7 runtime.

Master uses the exported catalog inertia, rated torque, speed limit and encoder
counts. Torque-speed shape, controller gains, installed encoder accuracy, tool
inertia and the entire C-axis plant remain assumptions. No ISO grade prediction.
"""
import argparse
import copy
import json
import math

from analyze import HERE, arcsec_radians, omega, study_curve_torque


def simulate(inputs, catalog, loop_hz=4000, predict=True, jitter_us=None, force_n=None):
    h, plant = inputs["hobber"], inputs["hobber_simulation"]
    motor = next(row for row in catalog["motors"] if row["spec"]["designation"] == plant["master_motor_designation"])
    spec = motor["spec"]
    curve = motor.get("study_continuous_curve")
    if curve is None:
        raise ValueError("Simulation requires an explicitly identified variant curve; operating conditions remain assumed")
    rpm = h["reference_hob_rpm"]
    if rpm > spec["maxRpm"]:
        raise ValueError("Requested master speed exceeds the local motor model")
    jitter_us = h["residual_timestamp_error_us_bound"] if jitter_us is None else jitter_us
    force_n = plant["tangential_force_n"] if force_n is None else force_n
    period = 1 / loop_hz
    substeps = plant["integration_substeps"]
    dt = period / substeps
    count = round(plant["duration_s"] / dt)
    delay = h["prediction_horizon_us"] * 1e-6
    ratio = h["hob_starts"] / h["teeth"]
    radius_um = h["module_mm"] * h["teeth"] * 500
    inertia_b = spec["rotorInertia"] + plant["hob_load_inertia_kg_m2"]
    inertia_c = plant["table_inertia_kg_m2"]
    desired_b_velocity = omega(rpm)
    # Assumed critically damped speed PI loop; no use of the catalog peak torque.
    bandwidth_b = math.tau * plant["master_speed_bandwidth_hz"]
    kp_b = 2 * inertia_b * bandwidth_b
    ki_b = inertia_b * bandwidth_b**2
    bandwidth_c = math.tau * plant["table_bandwidth_hz"]
    kp_c = inertia_c * bandwidth_c**2
    kd_c = 2 * plant["table_damping_ratio"] * inertia_c * bandwidth_c
    theta_b, velocity_b = 0.0, desired_b_velocity
    theta_c, velocity_c = 0.0, desired_b_velocity * ratio
    integral_b = 0.0
    torque_b = torque_c = 0.0
    master_step = math.tau / motor["encoder_counts_per_rev"]
    table_step = math.tau / h["table_encoder_counts_per_rev"]
    angle_history = [theta_b]
    target_phase = 0.0
    target_velocity = desired_b_velocity * ratio
    command_time = 0.0
    previous_measurement = None
    previous_sample_time = None
    error_samples, trace = [], []
    saturated_b = saturated_c = evaluated_steps = 0
    min_rpm = rpm
    max_rpm = rpm
    stalled = False
    for step in range(count):
        t = step * dt
        if step % substeps == 0:
            sample_time = t - delay
            if sample_time < 0:
                sample_angle = sample_time * desired_b_velocity
            else:
                index = sample_time / dt
                lo = min(int(index), len(angle_history)-1)
                hi = min(lo+1, len(angle_history)-1)
                sample_angle = angle_history[lo] + (index-lo)*(angle_history[hi]-angle_history[lo])
            # Installed error and quantization are modeled independently of resolution.
            sample_angle += arcsec_radians(h["hob_encoder_installed_accuracy_arcsec_bound"]) * math.sin(sample_angle)
            measured_b = round(sample_angle / master_step) * master_step
            measured_velocity_b = desired_b_velocity if previous_measurement is None else (
                (measured_b-previous_measurement)/(sample_time-previous_sample_time))
            jitter = jitter_us * 1e-6 * math.sin(math.tau * plant["jitter_frequency_hz"] * t)
            age = delay + jitter if predict else 0.0
            target_phase = ratio * (measured_b + measured_velocity_b * age)
            target_velocity = ratio * measured_velocity_b
            command_time = t
            previous_measurement, previous_sample_time = measured_b, sample_time

            error_b = desired_b_velocity - measured_velocity_b
            requested_b = kp_b * error_b + integral_b
            continuous_limit = study_curve_torque(curve,abs(velocity_b)*60/math.tau)
            if continuous_limit is None:
                raise ValueError("Master speed outside digitized continuous curve")
            continuous_limit = min(spec["ratedTorque"],continuous_limit)
            torque_b = max(-continuous_limit, min(continuous_limit, requested_b))
            # Conditional integration prevents winding up against the continuous torque limit.
            if abs(requested_b) <= continuous_limit or error_b * requested_b < 0:
                integral_b += ki_b * error_b * period

        # Reconstruct the velocity target between controller samples. The assumed fast
        # inner table loop uses measured angle and ideal velocity feedback (optimistic).
        c_measured = theta_c + arcsec_radians(h["table_encoder_installed_accuracy_arcsec_bound"]) * math.sin(theta_c)
        c_measured = round(c_measured / table_step) * table_step
        c_target = target_phase + target_velocity * (t-command_time)
        requested_c = kp_c * (c_target-c_measured) + kd_c * (target_velocity-velocity_c)
        c_limit = plant["assumed_table_continuous_torque_nm"]
        torque_c = max(-c_limit, min(c_limit, requested_c))
        load = 0.0
        if t >= plant["load_step_time_s"]:
            load = force_n * h["hob_diameter_mm"] / 2000 * (
                1 + plant["gash_torque_fraction"] * math.sin(h["hob_gashes"] * theta_b))
        acceleration_b = (torque_b-load)/inertia_b
        acceleration_c = torque_c/inertia_c
        theta_b += velocity_b * dt + 0.5 * acceleration_b * dt**2
        theta_c += velocity_c * dt + 0.5 * acceleration_c * dt**2
        velocity_b += acceleration_b * dt
        velocity_c += acceleration_c * dt
        if velocity_b <= 0:
            # Terminate at stall: continuing the constant cutting-load scenario into
            # reverse rotation would not model the same cutting process.
            stalled = True
            min_rpm = 0.0
            break
        angle_history.append(theta_b)
        if t >= plant["evaluate_after_s"]:
            error_um = radius_um * (theta_c-ratio*theta_b)
            error_samples.append(error_um)
            evaluated_steps += 1
            saturated_b += abs(torque_b) >= continuous_limit * (1-1e-9)
            saturated_c += abs(torque_c) >= c_limit * (1-1e-9)
            actual_rpm = velocity_b * 60/math.tau
            min_rpm = min(min_rpm, actual_rpm)
            max_rpm = max(max_rpm, actual_rpm)
            if step % (substeps*4) == 0:
                trace.append({"time_s":t, "true_sync_error_um":error_um, "hob_rpm":actual_rpm})
    peak = max(abs(e) for e in error_samples)
    return {"motor":spec["designation"], "reference_hob_rpm":rpm, "loop_hz":loop_hz, "prediction_enabled":predict,
            "continuous_curve_id":curve["id"],"curve_status":curve["status"],
            "curve_operating_conditions_verified":curve["operatingConditionsVerified"],
            "timestamp_residual_us":jitter_us, "tangential_force_n":force_n,
            "peak_true_sync_error_um":peak,
            "rms_true_sync_error_um":math.sqrt(sum(e*e for e in error_samples)/len(error_samples)),
            "within_control_allocation_in_this_scenario":peak <= h["control_error_limit_um"],
            "minimum_hob_rpm":min_rpm, "maximum_hob_rpm":max_rpm,
            "within_catalog_speed_limit":max_rpm <= spec["maxRpm"],
            "master_stalled":stalled,
            "simulated_until_s":t,
            "master_continuous_torque_saturation_fraction":saturated_b/evaluated_steps,
            "assumed_table_torque_saturation_fraction":saturated_c/evaluated_steps,
            "trace":trace}


def run(inputs, catalog):
    cases = [simulate(inputs,catalog,hz,predict) for hz in inputs["hobber"]["loop_rate_samples_hz"] for predict in [False,True]]
    cases.append(simulate(inputs,catalog,8000,True,jitter_us=10))
    high_speed = copy.deepcopy(inputs)
    high_speed["hobber"]["reference_hob_rpm"] = 4500
    cases.append(simulate(high_speed,catalog,8000,True))
    cases.append(simulate(inputs,catalog,8000,True,force_n=400))
    return {"status":"Reduced-order local-model study; assumptions are not verified controller/plant performance",
            "master_catalog":inputs["local_catalog"], "plant_assumptions":inputs["hobber_simulation"],
            "limitations":["No physical C-table motor/drive model exists yet; C inertia, torque and gains are assumed",
                           "B torque uses a matched-variant digitized continuous curve with reading allowance; operating conditions are unverified",
                           "The local motor encoder resolution does not prove timely access to its counts",
                           "Ideal inner table velocity feedback; no torsion, cogging, filters, packet loss or thermal model",
                           "3000 and 4500 rpm scenarios only; no 8000 rpm direct-drive local candidate exists",
                           "Control phase error is not a predicted ISO gear grade"],
            "cases":cases}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check",action="store_true")
    parser.add_argument("--plot",action="store_true")
    args = parser.parse_args()
    inputs = json.loads((HERE/"requirements.json").read_text())
    catalog = json.loads((HERE/inputs["local_catalog"]).read_text())
    result = run(inputs,catalog)
    # No-prediction zero-load case independently recovers the known constant-speed lag.
    constant = simulate(inputs,catalog,8000,False,jitter_us=0,force_n=0)
    expected = inputs["hobber"]["module_mm"]*inputs["hobber"]["teeth"]*500 * omega(inputs["hobber"]["reference_hob_rpm"]) * inputs["hobber"]["hob_starts"]/inputs["hobber"]["teeth"] * inputs["hobber"]["prediction_horizon_us"]*1e-6
    assert abs(constant["rms_true_sync_error_um"]-expected) < 0.4
    assert result["cases"][-1]["master_continuous_torque_saturation_fraction"] > 0
    assert result["cases"][-1]["master_stalled"]
    encoded = json.dumps(result,indent=2,allow_nan=False)+"\n"
    path = HERE/"hob-simulation.json"
    if args.check:
        if path.read_text() != encoded:
            raise SystemExit("Hob simulation results are stale")
    else:
        path.write_text(encoded)
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1,2,figsize=(12,4.8),layout="constrained")
        for case in result["cases"]:
            if case["loop_hz"] != 8000:
                continue
            label = f'{case["reference_hob_rpm"]} rpm, predict={case["prediction_enabled"]}, {case["timestamp_residual_us"]} µs, {case["tangential_force_n"]} N'
            trace = case["trace"]
            ax = axes[0] if not case["prediction_enabled"] else axes[1]
            ax.plot([x["time_s"] for x in trace], [x["true_sync_error_um"] for x in trace],label=label)
        for ax in axes:
            ax.set(xlabel="Time (s)",ylabel="True phase error at work pitch circle (µm)")
            ax.grid(alpha=0.25)
            ax.legend(fontsize=7)
        axes[0].set_title("Uncompensated 250 µs master delay")
        axes[1].set_title("Prediction; assumed table plant; overload stops at stall")
        fig.savefig(HERE/"hob-simulation.png",dpi=160)
        plt.close(fig)
    for case in result["cases"]:
        print(f'{case["reference_hob_rpm"]} rpm, {case["loop_hz"]} Hz, prediction={case["prediction_enabled"]}, '
              f'jitter={case["timestamp_residual_us"]} us, force={case["tangential_force_n"]} N: '
              f'peak error {case["peak_true_sync_error_um"]:.3f} um, stalled={case["master_stalled"]}')


if __name__ == "__main__":
    main()
