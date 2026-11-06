#include "dq_controller.h"
#define DPRIME_NOM 0.5

void dq_init(DQController* ctrl, double Kp, double Ki, double Ts, double Vdc, double Ls)
{
    // Assign parameters
    ctrl->Kp = Kp;
    ctrl->Ki = Ki;
    ctrl->Ts = Ts;
    ctrl->Vdc = Vdc;
    ctrl->Ls = Ls;

    // Initialize states to zero
    ctrl->theta_e = 0.0;
    ctrl->id_err_prev = 0.0;
    ctrl->iq_err_prev = 0.0;
    ctrl->d_int_prev = 0.0;
    ctrl->q_int_prev = 0.0;
}

void dq_update(DQController* ctrl, 
               double iLA, double iLB, double iLC, 
               double id_ref, double iq_ref, double omega_e, 
               double* m_a, double* m_b, double* m_c, 
               double* Id_k, double* Iq_k, double* Vd_k, double* Vq_k)
{
    // Local intermediate variables
    double id, iq, id_err, iq_err, d_int, q_int, vd, vq, md, mq;

    // Reference angle generation
    ctrl->theta_e = ctrl->theta_e + omega_e * ctrl->Ts;
    if (ctrl->theta_e > 2 * PI) {
        ctrl->theta_e = ctrl->theta_e - 2 * PI;
    }

    /**********************************************************************
     * TODO 1: Implement the amplitude-invariant abc-to-dq transformation.
     *
     * Use the measured phase currents iLA, iLB, iLC and the electrical
     * angle ctrl->theta_e to compute:
     *
     *   id
     *   iq
     *
     * Use the amplitude-invariant scaling factor:
     *
     *   2.0 / 3.0
     *
     * and the class convention:
     *
     *   id uses cosine terms
     *   iq uses negative sine terms
     *
     * Helpful reminder:
     *
     *   phase B angle = theta_e - OFFSET
     *   phase C angle = theta_e + OFFSET
     *
     **********************************************************************/
    
    /* STUDENT CODE STARTS HERE */

    id = (2.0 / 3.0) * (1.0 / DPRIME_NOM) * (iLA * cos(ctrl->theta_e) + iLB * cos(ctrl->theta_e - OFFSET) + iLC * cos(ctrl->theta_e + OFFSET));
    iq = (2.0 / 3.0) * (1.0 / DPRIME_NOM) * (-1.0) * (iLA * sin(ctrl->theta_e) + iLB * sin(ctrl->theta_e - OFFSET) + iLC * sin(ctrl->theta_e + OFFSET));

    /* STUDENT CODE ENDS HERE */

    id_err = id_ref - id;
    iq_err = iq_ref - iq;

    /**********************************************************************
     * TODO 2: Implement the trapezoidal PI integrators.
     *
     * Compute:
     *
     *   d_int
     *   q_int
     *
     * using:
     *   - current error
     *   - previous error
     *   - previous integrator state
     *   - ctrl->Ki
     *   - ctrl->Ts
     *
     * Do NOT use backward Euler here.
     *
     * Reminder:
     * trapezoidal integration uses the average of the current and previous
     * input values over one time step.
     *
     **********************************************************************/
    
    /* STUDENT CODE STARTS HERE */

    d_int = ctrl->d_int_prev + 0.5 * (id_err + ctrl->id_err_prev);
    q_int = ctrl->q_int_prev + 0.5 * (ctrl->iq_err_prev + iq_err);

    /* STUDENT CODE ENDS HERE */

    // Voltage calculation with cross-coupling decoupling
    vd = d_int + ctrl->Kp * id_err + omega_e * ctrl->Ls * iq;
    vq = q_int + ctrl->Kp * iq_err - omega_e * ctrl->Ls * id;

    // Modulation indices calculation
    md = vd * (2.0 / ctrl->Vdc);
    mq = vq * (2.0 / ctrl->Vdc);

    // Save current states for the next time step
    ctrl->id_err_prev = id_err;
    ctrl->iq_err_prev = iq_err;
    ctrl->d_int_prev = d_int;
    ctrl->q_int_prev = q_int;

    /**********************************************************************
     * TODO 3: Implement the dq-to-abc transformation for modulation.
     *
     * Use md and mq to compute:
     *
     *   *m_a
     *   *m_b
     *   *m_c
     *
     * Use the same electrical angle convention as above.
     *
     * Helpful reminder:
     *
     *   phase A uses theta_e
     *   phase B uses theta_e - OFFSET
     *   phase C uses theta_e + OFFSET
     *
     **********************************************************************/
    
    /* STUDENT CODE STARTS HERE */

    *m_a = md*cos(ctrl->theta_e) - mq*sin(ctrl->theta_e);
    *m_b = md*cos(ctrl->theta_e - OFFSET) - mq*sin(ctrl->theta_e - OFFSET);
    *m_c = md*cos(ctrl->theta_e + OFFSET) - mq*sin(ctrl->theta_e + OFFSET);

    /* STUDENT CODE ENDS HERE */

    // Duty cycle saturation limits
    if (*m_a >= 0.99) *m_a = 0.99;
    else if (*m_a <= -0.99) *m_a = -0.99;

    if (*m_b >= 0.99) *m_b = 0.99;
    else if (*m_b <= -0.99) *m_b = -0.99;

    if (*m_c >= 0.99) *m_c = 0.99;
    else if (*m_c <= -0.99) *m_c = -0.99;

    // Assign outputs
    *Id_k = id;
    *Iq_k = iq;
    *Vd_k = vd;
    *Vq_k = vq;
}