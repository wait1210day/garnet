#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
# SPDX-License-Identifier: GPL-3.0-or-later

from dataclasses import dataclass
from math import pi, log10, log2, ceil


# Digital 2p2z compensator:
#   H(z) = (b0 + b1*z^(-1) + b2*z^(-2)) / (1 - a1*z^(-1) - a2*z^(-2))
@dataclass
class Digital2p2zComp:
    b0: float
    b1: float
    b2: float
    a1: float
    a2: float

    def scale(self, k: float) -> Digital2p2zComp:
        return Digital2p2zComp(self.b0 * k, self.b1 * k, self.b2 * k, self.a1, self.a2)


# Type II analog compensator:
#   H(s) = (K_integrator / s) (1 + s/w_z1)/(1 + s/w_p1)
# Sample network: Rtop, Rbottom; opamp feedback network: (Cs + Rs) || Cp
@dataclass
class AnalogType2Comp:
    Rtop_kohm: float
    Rs_kohm: float
    Cs_nF: float
    Cp_nF: float


class Compensator:
    wp1: float
    wz1: float
    fp1_khz: float
    fz1_khz: float
    k_int: float
    k_int_db: float

    def __init__(self, wp1: float, wz1: float, k_int: float):
        # fp0 is just the low-frequency pole at 0Hz
        self.wp1 = wp1
        self.wz1 = wz1
        self.fp1_khz = wp1 / (2*pi) / 1000
        self.fz1_khz = wz1 / (2*pi) / 1000
        self.k_int = k_int
        self.k_int_db = 20 * log10(k_int)

    @staticmethod
    def from_freqs(fp1_khz: float, fz1_khz: float, k_int: float) -> Compensator:
        return Compensator(fp1_khz * 1000 * 2*pi, fz1_khz * 1000 * 2*pi, k_int)

    @staticmethod
    def from_analog(params: AnalogType2Comp) -> Compensator:
        # wp1 = 1 / (Rs * (Cs || Cp))
        wp1 = 1e6 * (params.Cs_nF + params.Cp_nF) / (params.Cs_nF * params.Cp_nF * params.Rs_kohm)
        wz1 = 1e6 * 1 / (params.Rs_kohm * params.Cs_nF)
        k_int = 1e6 * 1 / (params.Rtop_kohm * (params.Cs_nF + params.Cp_nF))
        return Compensator(wp1, wz1, k_int)

    def solve_analog(self, Rtop_kohm: float) -> AnalogType2Comp:
        Csum_nF = 1e6 * 1 / (self.k_int * Rtop_kohm)
        Cp_nF = Csum_nF * (self.wz1 / self.wp1)
        Cs_nF = Csum_nF - Cp_nF
        Rs_kohm = 1e6 * 1 / (self.wz1 * Cs_nF)
        return AnalogType2Comp(Rtop_kohm, Rs_kohm, Cs_nF, Cp_nF)

    def solve_2p2z(self, fs_hz: float) -> Digital2p2zComp:
        # Tustin transform constant: s = c * (1 - z^-1) / (1 + z^-1)
        c = 2.0 * fs_hz

        # Extract continuous domain parameters
        wp = self.wp1
        wz = self.wz1
        k_int = self.k_int

        # Calculate the normalization factor for the denominator
        # A0 = c * (1 + c/wp)
        den = c * (1.0 + c / wp)

        # Calculate numerator coefficients (b0, b1, b2)
        b0 = (k_int * (1.0 + c / wz)) / den
        b1 = (2.0 * k_int) / den
        b2 = (k_int * (1.0 - c / wz)) / den

        # Calculate denominator coefficients (a1, a2)
        # Note: The target transfer function format is 1 - a1*z^(-1) - a2*z^(-2)
        a1 = (2.0 * c) / (c + wp)
        a2 = (wp - c) / (c + wp)

        return Digital2p2zComp(b0, b1, b2, a1, a2)

    def bode(self, filename: str, f_start_hz: float=10, f_stop_hz: float=1e6, points: int=1000):
        import matplotlib.pyplot as plt
        import control as ct
        import numpy as np

        num = [self.k_int / self.wz1, self.k_int]
        den = [1 / self.wp1, 1, 0]
        sys = ct.TransferFunction(num, den)

        w = np.logspace(np.log10(f_start_hz * 2 * np.pi),
                        np.log10(f_stop_hz * 2 * np.pi),
                        points)

        plt.figure(figsize=(10, 8))

        ct.bode_plot(sys, omega=w, dB=True, Hz=True, grid=True)

        axs = plt.gcf().get_axes()
        axs[0].set_title(f"Type II Compensator Bode Plot\n"
                         f"$f_{{z1}}$={self.fz1_khz:.2f}kHz, "
                         f"$f_{{p1}}$={self.fp1_khz:.2f}kHz, "
                         f"$K_{{int}}$={self.k_int_db:.1f}dB")

        axs[0].axvline(self.fz1_khz * 1000, color='r', linestyle='--', alpha=0.5, label='Zero')
        axs[0].axvline(self.fp1_khz * 1000, color='g', linestyle='--', alpha=0.5, label='Pole')
        axs[0].legend()

        plt.tight_layout()
        plt.savefig(filename)


@dataclass
class CodeGenOptions:
    sym_name_b0: str = 'B0'
    sym_name_b1: str = 'B1'
    sym_name_b2: str = 'B2'
    sym_name_a1: str = 'A1'
    sym_name_a2: str = 'A2'
    sym_acc_bits: str = 'ACC_BITS'
    sym_q_bits: str = 'Q_BITS'

    @staticmethod
    def from_prefix(prefix: str):
        return CodeGenOptions(
            prefix + '_B0', prefix + '_B1', prefix + '_B2', prefix + '_A1', prefix + '_A2',
            prefix + '_ACC_BITS', prefix + '_Q_BITS'
        )


@dataclass
class QFormat2p2zComp:
    accumulator_bits: int
    q_bits: int
    b0: int
    b1: int
    b2: int
    a1: int
    a2: int

    def gen_coeffs_macro(self, options: CodeGenOptions = CodeGenOptions()):
        return f'#define {options.sym_name_b0}  ({self.b0})\n' + \
               f'#define {options.sym_name_b1}  ({self.b1})\n' + \
               f'#define {options.sym_name_b2}  ({self.b2})\n' + \
               f'#define {options.sym_name_a1}  ({self.a1})\n' + \
               f'#define {options.sym_name_a2}  ({self.a2})\n' + \
               f'#define {options.sym_acc_bits} {self.accumulator_bits}\n' + \
               f'#define {options.sym_q_bits}   {self.q_bits}'


class Quantifier:
    accumulator_bits: int

    def __init__(self, accumulator_bits: int):
        if accumulator_bits % 2 != 0:
            raise RuntimeError('invalid accumulator_bits')

        self.accumulator_bits = accumulator_bits

    def best_fit_coeff_width_bits(self) -> int:
        # The common implementation of IIR filter contains multiply-and-accumulate operations:
        #   acc += b0 * err
        #   acc += b1 * e_n1
        #   ...
        # To prevent `acc` from overflow, all the coefficients must be half width.
        return self.accumulator_bits >> 1

    def best_fit_q_bits(self, comp: Digital2p2zComp) -> int:
        max_abs = max(abs(comp.b0), abs(comp.b1), abs(comp.b2), abs(comp.a1), abs(comp.a2))
        # +1 for the sign bit
        integer_bits = ceil(log2(max_abs + 1)) + 1
        coeff_bits = self.best_fit_coeff_width_bits()
        if coeff_bits < integer_bits:
            raise RuntimeError(f'coefficients cannot be quantified with {self.accumulator_bits}bit accumulator')

        return coeff_bits - integer_bits

    def to_q_format(self, comp: Digital2p2zComp, q_bits: int, total_bits: int) -> QFormat2p2zComp:
        scalar = 1 << q_bits
        max_value = (1 << (total_bits - 1)) - 1
        min_value = -(1 << (total_bits - 1))

        # Check for overflow
        coeffs = {
            "b0": comp.b0,
            "b1": comp.b1,
            "b2": comp.b2,
            "a1": comp.a1,
            "a2": comp.a2,
        }
        for key, value in coeffs.items():
            fixed_value = int(round(value * scalar))
            if fixed_value > max_value or fixed_value < min_value:
                raise RuntimeError(f'coefficient {key} overflow: {fixed_value} is out of range [{min_value}, {max_value}]')
            coeffs[key] = fixed_value

        return QFormat2p2zComp(accumulator_bits=self.accumulator_bits, q_bits=q_bits, **coeffs)

    def to_best_q_format(self, comp: Digital2p2zComp) -> QFormat2p2zComp:
        return self.to_q_format(comp, self.best_fit_q_bits(comp), self.best_fit_coeff_width_bits())


comp_cv = Compensator.from_freqs(150, 2, 242000 / 5)
# print(comp_cv.solve_analog(90))
cv_2p2z = comp_cv.solve_2p2z(fs_hz=250_000)
print(f'struct iir_2p2z_coeffs cv_2p2z_slow = {{')
print(f'    .b0 = {cv_2p2z.b0:.06f}f,')
print(f'    .b1 = {cv_2p2z.b1:.06f}f,')
print(f'    .b2 = {cv_2p2z.b2:.06f}f,')
print(f'    .a1 = {cv_2p2z.a1:.06f}f,')
print(f'    .a2 = {cv_2p2z.a2:.06f}f')
print(f'}};')

comp_cv_fast = Compensator.from_freqs(200, 5, 242000 / 1)
#comp_cv_fast = comp_cv
# print(comp_cv.solve_analog(90))
cv_2p2z_fast = comp_cv_fast.solve_2p2z(fs_hz=250_000)
print(f'struct iir_2p2z_coeffs cv_2p2z_fast = {{')
print(f'    .b0 = {cv_2p2z_fast.b0:.06f}f,')
print(f'    .b1 = {cv_2p2z_fast.b1:.06f}f,')
print(f'    .b2 = {cv_2p2z_fast.b2:.06f}f,')
print(f'    .a1 = {cv_2p2z_fast.a1:.06f}f,')
print(f'    .a2 = {cv_2p2z_fast.a2:.06f}f')
print(f'}};')

boost_comp_cv = Compensator.from_freqs(100, 1, 37000)
boost_cv_2p2z = boost_comp_cv.solve_2p2z(fs_hz=250_000)
print(f'.b0 = {boost_cv_2p2z.b0:.06f}f,')
print(f'.b1 = {boost_cv_2p2z.b1:.06f}f,')
print(f'.b2 = {boost_cv_2p2z.b2:.06f}f,')
print(f'.a1 = {boost_cv_2p2z.a1:.06f}f,')
print(f'.a2 = {boost_cv_2p2z.a2:.06f}f')

# print(Compensator.from_analog(AnalogType2Comp(90, 50, 10, 0.68)).__dict__)
