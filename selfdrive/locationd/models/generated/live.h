#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_398726393068994039);
void live_err_fun(double *nom_x, double *delta_x, double *out_7226699364716968282);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3146457545800539540);
void live_H_mod_fun(double *state, double *out_7216981441376091161);
void live_f_fun(double *state, double dt, double *out_3387687771793490306);
void live_F_fun(double *state, double dt, double *out_2725999673171291231);
void live_h_4(double *state, double *unused, double *out_8616059814145978514);
void live_H_4(double *state, double *unused, double *out_4559519035519719826);
void live_h_9(double *state, double *unused, double *out_1324040322580831860);
void live_H_9(double *state, double *unused, double *out_6600006102925384320);
void live_h_10(double *state, double *unused, double *out_8744657120921916929);
void live_H_10(double *state, double *unused, double *out_3840710102944076896);
void live_h_12(double *state, double *unused, double *out_4204782893406516894);
void live_H_12(double *state, double *unused, double *out_1821739341523013170);
void live_h_35(double *state, double *unused, double *out_2956882728595847292);
void live_H_35(double *state, double *unused, double *out_7926181092892327202);
void live_h_32(double *state, double *unused, double *out_4545560158010009988);
void live_H_32(double *state, double *unused, double *out_8579724643484215613);
void live_h_13(double *state, double *unused, double *out_665213163273114834);
void live_H_13(double *state, double *unused, double *out_5576902823318934147);
void live_h_14(double *state, double *unused, double *out_1324040322580831860);
void live_H_14(double *state, double *unused, double *out_6600006102925384320);
void live_h_33(double *state, double *unused, double *out_7989437974873408495);
void live_H_33(double *state, double *unused, double *out_7370005976178366810);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}