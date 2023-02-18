#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5849170583003101836) {
   out_5849170583003101836[0] = delta_x[0] + nom_x[0];
   out_5849170583003101836[1] = delta_x[1] + nom_x[1];
   out_5849170583003101836[2] = delta_x[2] + nom_x[2];
   out_5849170583003101836[3] = delta_x[3] + nom_x[3];
   out_5849170583003101836[4] = delta_x[4] + nom_x[4];
   out_5849170583003101836[5] = delta_x[5] + nom_x[5];
   out_5849170583003101836[6] = delta_x[6] + nom_x[6];
   out_5849170583003101836[7] = delta_x[7] + nom_x[7];
   out_5849170583003101836[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7430193903482424264) {
   out_7430193903482424264[0] = -nom_x[0] + true_x[0];
   out_7430193903482424264[1] = -nom_x[1] + true_x[1];
   out_7430193903482424264[2] = -nom_x[2] + true_x[2];
   out_7430193903482424264[3] = -nom_x[3] + true_x[3];
   out_7430193903482424264[4] = -nom_x[4] + true_x[4];
   out_7430193903482424264[5] = -nom_x[5] + true_x[5];
   out_7430193903482424264[6] = -nom_x[6] + true_x[6];
   out_7430193903482424264[7] = -nom_x[7] + true_x[7];
   out_7430193903482424264[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5165661960782214543) {
   out_5165661960782214543[0] = 1.0;
   out_5165661960782214543[1] = 0;
   out_5165661960782214543[2] = 0;
   out_5165661960782214543[3] = 0;
   out_5165661960782214543[4] = 0;
   out_5165661960782214543[5] = 0;
   out_5165661960782214543[6] = 0;
   out_5165661960782214543[7] = 0;
   out_5165661960782214543[8] = 0;
   out_5165661960782214543[9] = 0;
   out_5165661960782214543[10] = 1.0;
   out_5165661960782214543[11] = 0;
   out_5165661960782214543[12] = 0;
   out_5165661960782214543[13] = 0;
   out_5165661960782214543[14] = 0;
   out_5165661960782214543[15] = 0;
   out_5165661960782214543[16] = 0;
   out_5165661960782214543[17] = 0;
   out_5165661960782214543[18] = 0;
   out_5165661960782214543[19] = 0;
   out_5165661960782214543[20] = 1.0;
   out_5165661960782214543[21] = 0;
   out_5165661960782214543[22] = 0;
   out_5165661960782214543[23] = 0;
   out_5165661960782214543[24] = 0;
   out_5165661960782214543[25] = 0;
   out_5165661960782214543[26] = 0;
   out_5165661960782214543[27] = 0;
   out_5165661960782214543[28] = 0;
   out_5165661960782214543[29] = 0;
   out_5165661960782214543[30] = 1.0;
   out_5165661960782214543[31] = 0;
   out_5165661960782214543[32] = 0;
   out_5165661960782214543[33] = 0;
   out_5165661960782214543[34] = 0;
   out_5165661960782214543[35] = 0;
   out_5165661960782214543[36] = 0;
   out_5165661960782214543[37] = 0;
   out_5165661960782214543[38] = 0;
   out_5165661960782214543[39] = 0;
   out_5165661960782214543[40] = 1.0;
   out_5165661960782214543[41] = 0;
   out_5165661960782214543[42] = 0;
   out_5165661960782214543[43] = 0;
   out_5165661960782214543[44] = 0;
   out_5165661960782214543[45] = 0;
   out_5165661960782214543[46] = 0;
   out_5165661960782214543[47] = 0;
   out_5165661960782214543[48] = 0;
   out_5165661960782214543[49] = 0;
   out_5165661960782214543[50] = 1.0;
   out_5165661960782214543[51] = 0;
   out_5165661960782214543[52] = 0;
   out_5165661960782214543[53] = 0;
   out_5165661960782214543[54] = 0;
   out_5165661960782214543[55] = 0;
   out_5165661960782214543[56] = 0;
   out_5165661960782214543[57] = 0;
   out_5165661960782214543[58] = 0;
   out_5165661960782214543[59] = 0;
   out_5165661960782214543[60] = 1.0;
   out_5165661960782214543[61] = 0;
   out_5165661960782214543[62] = 0;
   out_5165661960782214543[63] = 0;
   out_5165661960782214543[64] = 0;
   out_5165661960782214543[65] = 0;
   out_5165661960782214543[66] = 0;
   out_5165661960782214543[67] = 0;
   out_5165661960782214543[68] = 0;
   out_5165661960782214543[69] = 0;
   out_5165661960782214543[70] = 1.0;
   out_5165661960782214543[71] = 0;
   out_5165661960782214543[72] = 0;
   out_5165661960782214543[73] = 0;
   out_5165661960782214543[74] = 0;
   out_5165661960782214543[75] = 0;
   out_5165661960782214543[76] = 0;
   out_5165661960782214543[77] = 0;
   out_5165661960782214543[78] = 0;
   out_5165661960782214543[79] = 0;
   out_5165661960782214543[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2341687151300141858) {
   out_2341687151300141858[0] = state[0];
   out_2341687151300141858[1] = state[1];
   out_2341687151300141858[2] = state[2];
   out_2341687151300141858[3] = state[3];
   out_2341687151300141858[4] = state[4];
   out_2341687151300141858[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2341687151300141858[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2341687151300141858[7] = state[7];
   out_2341687151300141858[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5047462852627777116) {
   out_5047462852627777116[0] = 1;
   out_5047462852627777116[1] = 0;
   out_5047462852627777116[2] = 0;
   out_5047462852627777116[3] = 0;
   out_5047462852627777116[4] = 0;
   out_5047462852627777116[5] = 0;
   out_5047462852627777116[6] = 0;
   out_5047462852627777116[7] = 0;
   out_5047462852627777116[8] = 0;
   out_5047462852627777116[9] = 0;
   out_5047462852627777116[10] = 1;
   out_5047462852627777116[11] = 0;
   out_5047462852627777116[12] = 0;
   out_5047462852627777116[13] = 0;
   out_5047462852627777116[14] = 0;
   out_5047462852627777116[15] = 0;
   out_5047462852627777116[16] = 0;
   out_5047462852627777116[17] = 0;
   out_5047462852627777116[18] = 0;
   out_5047462852627777116[19] = 0;
   out_5047462852627777116[20] = 1;
   out_5047462852627777116[21] = 0;
   out_5047462852627777116[22] = 0;
   out_5047462852627777116[23] = 0;
   out_5047462852627777116[24] = 0;
   out_5047462852627777116[25] = 0;
   out_5047462852627777116[26] = 0;
   out_5047462852627777116[27] = 0;
   out_5047462852627777116[28] = 0;
   out_5047462852627777116[29] = 0;
   out_5047462852627777116[30] = 1;
   out_5047462852627777116[31] = 0;
   out_5047462852627777116[32] = 0;
   out_5047462852627777116[33] = 0;
   out_5047462852627777116[34] = 0;
   out_5047462852627777116[35] = 0;
   out_5047462852627777116[36] = 0;
   out_5047462852627777116[37] = 0;
   out_5047462852627777116[38] = 0;
   out_5047462852627777116[39] = 0;
   out_5047462852627777116[40] = 1;
   out_5047462852627777116[41] = 0;
   out_5047462852627777116[42] = 0;
   out_5047462852627777116[43] = 0;
   out_5047462852627777116[44] = 0;
   out_5047462852627777116[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5047462852627777116[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5047462852627777116[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5047462852627777116[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5047462852627777116[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5047462852627777116[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5047462852627777116[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5047462852627777116[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5047462852627777116[53] = -9.8000000000000007*dt;
   out_5047462852627777116[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5047462852627777116[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5047462852627777116[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5047462852627777116[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5047462852627777116[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5047462852627777116[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5047462852627777116[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5047462852627777116[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5047462852627777116[62] = 0;
   out_5047462852627777116[63] = 0;
   out_5047462852627777116[64] = 0;
   out_5047462852627777116[65] = 0;
   out_5047462852627777116[66] = 0;
   out_5047462852627777116[67] = 0;
   out_5047462852627777116[68] = 0;
   out_5047462852627777116[69] = 0;
   out_5047462852627777116[70] = 1;
   out_5047462852627777116[71] = 0;
   out_5047462852627777116[72] = 0;
   out_5047462852627777116[73] = 0;
   out_5047462852627777116[74] = 0;
   out_5047462852627777116[75] = 0;
   out_5047462852627777116[76] = 0;
   out_5047462852627777116[77] = 0;
   out_5047462852627777116[78] = 0;
   out_5047462852627777116[79] = 0;
   out_5047462852627777116[80] = 1;
}
void h_25(double *state, double *unused, double *out_3381307905191584923) {
   out_3381307905191584923[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8710627801442850824) {
   out_8710627801442850824[0] = 0;
   out_8710627801442850824[1] = 0;
   out_8710627801442850824[2] = 0;
   out_8710627801442850824[3] = 0;
   out_8710627801442850824[4] = 0;
   out_8710627801442850824[5] = 0;
   out_8710627801442850824[6] = 1;
   out_8710627801442850824[7] = 0;
   out_8710627801442850824[8] = 0;
}
void h_24(double *state, double *unused, double *out_4646091554282428615) {
   out_4646091554282428615[0] = state[4];
   out_4646091554282428615[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7563466673261201226) {
   out_7563466673261201226[0] = 0;
   out_7563466673261201226[1] = 0;
   out_7563466673261201226[2] = 0;
   out_7563466673261201226[3] = 0;
   out_7563466673261201226[4] = 1;
   out_7563466673261201226[5] = 0;
   out_7563466673261201226[6] = 0;
   out_7563466673261201226[7] = 0;
   out_7563466673261201226[8] = 0;
   out_7563466673261201226[9] = 0;
   out_7563466673261201226[10] = 0;
   out_7563466673261201226[11] = 0;
   out_7563466673261201226[12] = 0;
   out_7563466673261201226[13] = 0;
   out_7563466673261201226[14] = 1;
   out_7563466673261201226[15] = 0;
   out_7563466673261201226[16] = 0;
   out_7563466673261201226[17] = 0;
}
void h_30(double *state, double *unused, double *out_2779889747578221369) {
   out_2779889747578221369[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6192294842935602197) {
   out_6192294842935602197[0] = 0;
   out_6192294842935602197[1] = 0;
   out_6192294842935602197[2] = 0;
   out_6192294842935602197[3] = 0;
   out_6192294842935602197[4] = 1;
   out_6192294842935602197[5] = 0;
   out_6192294842935602197[6] = 0;
   out_6192294842935602197[7] = 0;
   out_6192294842935602197[8] = 0;
}
void h_26(double *state, double *unused, double *out_4470337582313902329) {
   out_4470337582313902329[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5994612953392644568) {
   out_5994612953392644568[0] = 0;
   out_5994612953392644568[1] = 0;
   out_5994612953392644568[2] = 0;
   out_5994612953392644568[3] = 0;
   out_5994612953392644568[4] = 0;
   out_5994612953392644568[5] = 0;
   out_5994612953392644568[6] = 0;
   out_5994612953392644568[7] = 1;
   out_5994612953392644568[8] = 0;
}
void h_27(double *state, double *unused, double *out_8382981172852430568) {
   out_8382981172852430568[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3968700771751658980) {
   out_3968700771751658980[0] = 0;
   out_3968700771751658980[1] = 0;
   out_3968700771751658980[2] = 0;
   out_3968700771751658980[3] = 1;
   out_3968700771751658980[4] = 0;
   out_3968700771751658980[5] = 0;
   out_3968700771751658980[6] = 0;
   out_3968700771751658980[7] = 0;
   out_3968700771751658980[8] = 0;
}
void h_29(double *state, double *unused, double *out_8658175235136936457) {
   out_8658175235136936457[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5682063498621210013) {
   out_5682063498621210013[0] = 0;
   out_5682063498621210013[1] = 1;
   out_5682063498621210013[2] = 0;
   out_5682063498621210013[3] = 0;
   out_5682063498621210013[4] = 0;
   out_5682063498621210013[5] = 0;
   out_5682063498621210013[6] = 0;
   out_5682063498621210013[7] = 0;
   out_5682063498621210013[8] = 0;
}
void h_28(double *state, double *unused, double *out_4837855685021796101) {
   out_4837855685021796101[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7682281558018811029) {
   out_7682281558018811029[0] = 1;
   out_7682281558018811029[1] = 0;
   out_7682281558018811029[2] = 0;
   out_7682281558018811029[3] = 0;
   out_7682281558018811029[4] = 0;
   out_7682281558018811029[5] = 0;
   out_7682281558018811029[6] = 0;
   out_7682281558018811029[7] = 0;
   out_7682281558018811029[8] = 0;
}
void h_31(double *state, double *unused, double *out_256141623062437427) {
   out_256141623062437427[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5368404851159293092) {
   out_5368404851159293092[0] = 0;
   out_5368404851159293092[1] = 0;
   out_5368404851159293092[2] = 0;
   out_5368404851159293092[3] = 0;
   out_5368404851159293092[4] = 0;
   out_5368404851159293092[5] = 0;
   out_5368404851159293092[6] = 0;
   out_5368404851159293092[7] = 0;
   out_5368404851159293092[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_5849170583003101836) {
  err_fun(nom_x, delta_x, out_5849170583003101836);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7430193903482424264) {
  inv_err_fun(nom_x, true_x, out_7430193903482424264);
}
void car_H_mod_fun(double *state, double *out_5165661960782214543) {
  H_mod_fun(state, out_5165661960782214543);
}
void car_f_fun(double *state, double dt, double *out_2341687151300141858) {
  f_fun(state,  dt, out_2341687151300141858);
}
void car_F_fun(double *state, double dt, double *out_5047462852627777116) {
  F_fun(state,  dt, out_5047462852627777116);
}
void car_h_25(double *state, double *unused, double *out_3381307905191584923) {
  h_25(state, unused, out_3381307905191584923);
}
void car_H_25(double *state, double *unused, double *out_8710627801442850824) {
  H_25(state, unused, out_8710627801442850824);
}
void car_h_24(double *state, double *unused, double *out_4646091554282428615) {
  h_24(state, unused, out_4646091554282428615);
}
void car_H_24(double *state, double *unused, double *out_7563466673261201226) {
  H_24(state, unused, out_7563466673261201226);
}
void car_h_30(double *state, double *unused, double *out_2779889747578221369) {
  h_30(state, unused, out_2779889747578221369);
}
void car_H_30(double *state, double *unused, double *out_6192294842935602197) {
  H_30(state, unused, out_6192294842935602197);
}
void car_h_26(double *state, double *unused, double *out_4470337582313902329) {
  h_26(state, unused, out_4470337582313902329);
}
void car_H_26(double *state, double *unused, double *out_5994612953392644568) {
  H_26(state, unused, out_5994612953392644568);
}
void car_h_27(double *state, double *unused, double *out_8382981172852430568) {
  h_27(state, unused, out_8382981172852430568);
}
void car_H_27(double *state, double *unused, double *out_3968700771751658980) {
  H_27(state, unused, out_3968700771751658980);
}
void car_h_29(double *state, double *unused, double *out_8658175235136936457) {
  h_29(state, unused, out_8658175235136936457);
}
void car_H_29(double *state, double *unused, double *out_5682063498621210013) {
  H_29(state, unused, out_5682063498621210013);
}
void car_h_28(double *state, double *unused, double *out_4837855685021796101) {
  h_28(state, unused, out_4837855685021796101);
}
void car_H_28(double *state, double *unused, double *out_7682281558018811029) {
  H_28(state, unused, out_7682281558018811029);
}
void car_h_31(double *state, double *unused, double *out_256141623062437427) {
  h_31(state, unused, out_256141623062437427);
}
void car_H_31(double *state, double *unused, double *out_5368404851159293092) {
  H_31(state, unused, out_5368404851159293092);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
