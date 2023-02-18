#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_5849170583003101836);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7430193903482424264);
void car_H_mod_fun(double *state, double *out_5165661960782214543);
void car_f_fun(double *state, double dt, double *out_2341687151300141858);
void car_F_fun(double *state, double dt, double *out_5047462852627777116);
void car_h_25(double *state, double *unused, double *out_3381307905191584923);
void car_H_25(double *state, double *unused, double *out_8710627801442850824);
void car_h_24(double *state, double *unused, double *out_4646091554282428615);
void car_H_24(double *state, double *unused, double *out_7563466673261201226);
void car_h_30(double *state, double *unused, double *out_2779889747578221369);
void car_H_30(double *state, double *unused, double *out_6192294842935602197);
void car_h_26(double *state, double *unused, double *out_4470337582313902329);
void car_H_26(double *state, double *unused, double *out_5994612953392644568);
void car_h_27(double *state, double *unused, double *out_8382981172852430568);
void car_H_27(double *state, double *unused, double *out_3968700771751658980);
void car_h_29(double *state, double *unused, double *out_8658175235136936457);
void car_H_29(double *state, double *unused, double *out_5682063498621210013);
void car_h_28(double *state, double *unused, double *out_4837855685021796101);
void car_H_28(double *state, double *unused, double *out_7682281558018811029);
void car_h_31(double *state, double *unused, double *out_256141623062437427);
void car_H_31(double *state, double *unused, double *out_5368404851159293092);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}