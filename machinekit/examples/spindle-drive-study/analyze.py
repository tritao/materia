#!/usr/bin/env python3
"""Reproduce provisional spindle sizing and bounded hobbing synchronization errors.

No cutting-force prediction, closed-loop simulation or ISO gear grading is performed.
Run with --check to check committed numerical results without rewriting them.
Run with --plot to also render the engineering comparison figure (requires matplotlib).
"""
import argparse
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent


def omega(rpm):
    return rpm * math.tau / 60


def arcsec_radians(value):
    return value * math.pi / (180 * 3600)


def mill_point(spec, rpm):
    # Constant torque below the assumed base speed; constant power above it.
    torque = spec["output_power_w"] / omega(max(spec["base_rpm"], rpm))
    return {"spindle_rpm": rpm, "output_torque_nm": torque,
            "output_power_w": torque * omega(rpm)}


def hob_budget(spec, rpm):
    radius_um = spec["module_mm"] * spec["teeth"] / 2 * 1000
    ratio = spec["hob_starts"] / spec["teeth"]
    table_rpm = rpm * ratio
    horizon_s = spec["prediction_horizon_us"] * 1e-6
    # Worst-case sum of absolute tangential position-error bounds at the pitch circle.
    # Timestamp residual and acceleration prediction error are separate effects;
    # prediction removes the known constant-velocity delay, not timestamp uncertainty.
    terms = {
        "table_encoder_installed_accuracy": radius_um * arcsec_radians(spec["table_encoder_installed_accuracy_arcsec_bound"]),
        "table_encoder_half_count": radius_um * math.pi / spec["table_encoder_counts_per_rev"],
        "hob_encoder_installed_accuracy_referred_to_table": radius_um * ratio * arcsec_radians(spec["hob_encoder_installed_accuracy_arcsec_bound"]),
        "hob_encoder_half_count_referred_to_table": radius_um * ratio * math.pi / spec["hob_encoder_counts_per_rev"],
        "residual_timestamp_error": radius_um * omega(table_rpm) * spec["residual_timestamp_error_us_bound"] * 1e-6,
        "constant_velocity_prediction_error": radius_um * ratio * 0.5 * omega(spec["unmodeled_hob_acceleration_rpm_per_s_bound"]) * horizon_s**2,
        "table_following_error": radius_um * arcsec_radians(spec["table_following_error_arcsec_bound"]),
        "unmeasured_torsion": radius_um * arcsec_radians(spec["unmeasured_torsion_arcsec_bound"]),
    }
    total = sum(terms.values())
    return {"hob_rpm": rpm, "table_rpm": table_rpm,
            "gash_frequency_hz": rpm / 60 * spec["hob_gashes"],
            "terms_um_absolute_bound": terms,
            "sum_um_absolute_bound": total,
            "remaining_control_allocation_um": spec["control_error_limit_um"] - total,
            "within_provisional_control_allocation": total <= spec["control_error_limit_um"],
            "maximum_uncompensated_delay_us_for_entire_allocation": spec["control_error_limit_um"] / (radius_um * omega(table_rpm)) * 1e6,
            "uncompensated_delay_error_um": {str(us): radius_um * omega(table_rpm) * us * 1e-6
                                             for us in spec["uncompensated_latency_samples_us"]},
            "controller_samples_per_gash": {str(hz): hz / (rpm / 60 * spec["hob_gashes"])
                                             for hz in spec["loop_rate_samples_hz"]}}


def analyze(inputs):
    mill, hob = inputs["mill"], inputs["hobber"]
    if not 0 < mill["minimum_rpm"] <= mill["base_rpm"] <= mill["maximum_rpm"]:
        raise ValueError("Invalid mill speed range")
    if not 0 < mill["transmission_efficiency"] <= 1:
        raise ValueError("Invalid transmission efficiency")
    for key in ["output_power_w", "speed_regulation_fraction_target"]:
        if mill[key] <= 0:
            raise ValueError(f"Invalid mill {key}")
    for key, value in hob.items():
        if isinstance(value, (int, float)) and (not math.isfinite(value) or value <= 0):
            raise ValueError(f"Invalid hobber {key}")
    eta = mill["transmission_efficiency"]
    candidate_rows = []
    for candidate in inputs["leadshine_candidates"]:
        # Only the published rated point is used: no invented torque curve above base speed.
        ratio = mill["maximum_rpm"] / candidate["rated_rpm"]
        output_torque = candidate["rated_torque_nm"] * eta / ratio
        candidate_rows.append({**candidate,
            "spindle_to_motor_speed_ratio_at_rated_point": ratio,
            "spindle_output_torque_nm_at_8000_rpm_rated_motor_point": output_torque,
            "meets_mill_8000_rpm_output_point_only": output_torque >= mill_point(mill, mill["maximum_rpm"])["output_torque_nm"],
            "motor_rpm_at_mill_minimum": mill["minimum_rpm"] / ratio,
            "direct_drive_meets_hobber_maximum_speed": candidate["maximum_rpm"] >= hob["maximum_hob_rpm"],
            "qualification": "Unqualified: continuous curve, low-speed cooling, shaft load, drive and packaging still required"})
    force_rows = []
    for rpm in [hob["reference_hob_rpm"], hob["maximum_hob_rpm"]]:
        for force in hob["tangential_force_samples_n"]:
            torque = force * hob["hob_diameter_mm"] / 2000
            force_rows.append({"hob_rpm": rpm, "assumed_tangential_force_n": force,
                               "cutting_torque_nm": torque, "cutting_power_w": torque * omega(rpm)})
    return {
        "status": inputs["status"],
        "mill": {"required_motor_mechanical_power_w_before_transmission_losses": mill["output_power_w"] / eta,
                 "assumed_continuous_output_envelope": [mill_point(mill, rpm) for rpm in mill["sample_rpm"]],
                 "leadshine_rated_point_screen": candidate_rows},
        "hobber": {"radius_mm": hob["module_mm"] * hob["teeth"] / 2,
                   "hob_rpm_for_300_m_per_min": 300000 / (math.pi * hob["hob_diameter_mm"]),
                   "control_allocation_arcsec_at_pitch_circle": hob["control_error_limit_um"] / (hob["module_mm"] * hob["teeth"] * 500) * 180 * 3600 / math.pi,
                   "budgets": [hob_budget(hob, rpm) for rpm in [hob["reference_hob_rpm"], hob["maximum_hob_rpm"]]],
                   "tangential_force_sensitivity_not_a_load_prediction": force_rows,
                   "continuous_output_torque_nm_at_maximum_rpm_target": hob["spindle_output_power_w_target"] / omega(hob["maximum_hob_rpm"])}
    }


def verify(inputs, results):
    # Independent reference identities catch ratio, unit and loss-direction mistakes.
    reference_mill = {"output_power_w": 2200, "base_rpm": 3000}
    assert math.isclose(mill_point(reference_mill, 8000)["output_torque_nm"], 2.626056561016273, abs_tol=1e-12)
    for point in results["mill"]["assumed_continuous_output_envelope"]:
        assert math.isclose(point["output_power_w"], point["output_torque_nm"] * omega(point["spindle_rpm"]), abs_tol=1e-9)
    assert math.isclose(omega(15) * 0.001 * 30000, 47.12388980384689, abs_tol=1e-12)
    # One arcsecond at 30 mm is 0.145444... um, not an encoder count.
    assert math.isclose(30000 * arcsec_radians(2), 0.2908882086657216, abs_tol=1e-12)
    for budget in results["hobber"]["budgets"]:
        assert math.isclose(budget["table_rpm"] / budget["hob_rpm"], inputs["hobber"]["hob_starts"] / inputs["hobber"]["teeth"])
        assert math.isclose(budget["sum_um_absolute_bound"], sum(budget["terms_um_absolute_bound"].values()))
        assert budget["within_provisional_control_allocation"] == (budget["remaining_control_allocation_um"] >= 0)


def plot(inputs, results, destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout="constrained")
    speeds = list(range(500, 8001, 50))
    axes[0].plot(speeds, [mill_point(inputs["mill"], rpm)["output_torque_nm"] for rpm in speeds], linewidth=2.5)
    axes[0].set(xlabel="Spindle speed (rpm)", ylabel="Continuous output torque (Nm)",
                title="Mill: provisional 2.2 kW / 3000 rpm base envelope")
    axes[0].grid(alpha=0.25)
    budgets = results["hobber"]["budgets"]
    labels = ["Table accuracy", "Table quantization", "Hob accuracy / ratio", "Hob quantization / ratio",
              "Timestamp residual", "Prediction error", "Table following", "Torsion"]
    for j, b in enumerate(budgets):
        axes[1].barh([i+j*0.36 for i in range(len(labels))], list(b["terms_um_absolute_bound"].values()),
                     height=0.34, label=f'{b["hob_rpm"]} rpm: sum {b["sum_um_absolute_bound"]:.3f} µm')
    axes[1].set_yticks([i+0.18 for i in range(len(labels))], labels)
    axes[1].invert_yaxis()
    axes[1].set(xlabel="Absolute tangential error bound at r = 30 mm (µm)", title="Hobber: assumed bounds, 1 µm total allocation")
    axes[1].legend(fontsize=8)
    axes[1].grid(axis="x", alpha=0.25)
    fig.savefig(destination, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    inputs = json.loads((HERE / "requirements.json").read_text())
    results = analyze(inputs)
    verify(inputs, results)
    encoded = json.dumps(results, indent=2, allow_nan=False) + "\n"
    destination = HERE / "results.json"
    if args.check:
        if destination.read_text() != encoded:
            raise SystemExit("Spindle analysis results are stale; run analyze.py")
    else:
        destination.write_text(encoded)
    if args.plot:
        plot(inputs, results, HERE / "spindle-drive-study.png")
    print("Spindle requirements: calculation and consistency checks passed")
    for budget in results["hobber"]["budgets"]:
        print(f'Hob {budget["hob_rpm"]} rpm: {budget["sum_um_absolute_bound"]:.3f} µm bounded control error; '
              f'within allocation: {budget["within_provisional_control_allocation"]}')


if __name__ == "__main__":
    main()
