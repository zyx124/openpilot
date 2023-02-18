#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7319293501589091026);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_4162741171589677059);
void gnss_H_mod_fun(double *state, double *out_8461055172459071962);
void gnss_f_fun(double *state, double dt, double *out_731843756774827001);
void gnss_F_fun(double *state, double dt, double *out_6240084532272754259);
void gnss_h_6(double *state, double *sat_pos, double *out_3816338357172882771);
void gnss_H_6(double *state, double *sat_pos, double *out_5766712616144609008);
void gnss_h_20(double *state, double *sat_pos, double *out_3691888153521146197);
void gnss_H_20(double *state, double *sat_pos, double *out_3697786557749810575);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4150799050302153836);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_6034825300366711760);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4150799050302153836);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_6034825300366711760);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}