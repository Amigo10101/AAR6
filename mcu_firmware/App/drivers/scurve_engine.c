/*
 * scurve_engine.c
 *
 * S-Curve Motion Profile Engine
 * EXACT port from grotius-cnc/scurve_construct
 * Only change: double -> float, class -> functions
 */

#include "scurve_engine.h"
#include <math.h>

/*============================================================================
 * HELPER FUNCTIONS - Exact from grotius
 *============================================================================*/

static float ttot_period(scurve_period_t p) { return p.timend - p.timbeg; }

static float stot_period(scurve_period_t p) { return p.disend - p.disbeg; }

static float delvelocity(scurve_state_t *d) {
  float jm = d->jermax;
  float as = d->maxacc * 2.0f;
  float dvt = 2.0f * as / jm;
  float dv = fabsf((dvt * as) / 2.0f);
  return dv;
}

static void zero_period(scurve_period_t *p) {
  p->accbeg = 0;
  p->accend = 0;
  p->velini = 0;
  p->velbeg = 0;
  p->velend = 0;
  p->disbeg = 0;
  p->disend = 0;
  p->timbeg = 0;
  p->timend = 0;
  p->jermax = 0;
  p->accinf = 0;
}

/*============================================================================
 * PERIOD PLAYERS - Exact from grotius (called at 20kHz)
 *============================================================================*/

/* Play convex period t3 */
static void t3_play(float attime, float accinf, float jermax, float timbeg,
                    float timend, float velini, float disbeg, float *velend,
                    float *disend, float *accend) {
  float jm = jermax;
  float as = accinf;
  float ts = timbeg;
  float te = timend;
  float vo = velini;

  ts += attime;
  ts = fminf(ts, te);

  float vr = vo + as * ts - jm * (ts * ts) / 2.0f;
  float sr = vo * ts + as * (ts * ts) / 2.0f - jm * (ts * ts * ts) / 6.0f;
  float ar = as - jm * ts;

  *velend = vr;
  *disend = sr - disbeg;
  *accend = ar;
}

/* Play concave period t1 */
static void t1_play(float at_time, float accinf, float jermax, float timbeg,
                    float timend, float velini, float disbeg, float *velend,
                    float *disend, float *accend) {
  float jm = jermax;
  float ts = timbeg;
  float te = timend;
  float vo = velini;
  (void)accinf; /* unused in t1_play */

  ts += at_time;
  ts = fminf(ts, te);

  float vr = vo + jm * (ts * ts) / 2.0f;
  float sr = vo * ts + jm * (ts * ts * ts) / 6.0f;
  float ar = jm * ts;

  *velend = vr;
  *disend = sr - disbeg;
  *accend = ar;
}

/* Play linear acceleration period t2 */
static void t2_play(float attime, float accinf, float timsta, float timend,
                    float velbeg, float *velend, float *disend, float *accend) {
  float ts = timsta;
  float te = timend;
  float vo = velbeg;
  float as = accinf;

  ts += attime;
  ts = fminf(ts, te);

  float vr = vo + as * ts;
  float sr = (vr * vr - vo * vo) / (2.0f * as);
  float ar = as;

  *velend = vr;
  *disend = sr;
  *accend = ar;
}

/* Play steady period t4 */
static void t4_play(float attime, float timsta, float velbeg, float *velend,
                    float *disend, float *accend) {
  float ts = timsta;
  float vo = velbeg;

  ts += attime;

  *velend = vo;
  *disend = vo * ts;
  *accend = 0;
}

/*============================================================================
 * PERIOD BUILDERS - Exact from grotius (called once per move)
 *============================================================================*/

/* Build convex period t3 */
static void t3_build(float jermax, float accinf, float curvel, float curacc,
                     float endacc, scurve_period_t *c3) {
  float jm = jermax;
  float as = accinf;

  float ts = (as - curacc) / jm;
  float te = (as - endacc) / jm;

  float vo = curvel - as * ts + 0.5f * jm * (ts * ts);

  float vrs = vo + as * ts - jm * (ts * ts) / 2.0f;
  float srs = vo * ts + as * (ts * ts) / 2.0f - jm * (ts * ts * ts) / 6.0f;

  float vre = vo + as * te - jm * (te * te) / 2.0f;
  float sre = vo * te + as * (te * te) / 2.0f - jm * (te * te * te) / 6.0f;

  c3->disbeg = srs;
  c3->disend = sre;
  c3->velend = vre;
  c3->velini = vo;
  c3->accbeg = curacc;
  c3->accend = endacc;
  c3->timbeg = ts;
  c3->timend = te;
  c3->accinf = as;
  c3->jermax = jm;
}

/* Build concave period t1 */
static void t1_build(float jermax, float curvel, float curacc, float endacc,
                     scurve_period_t *c1) {
  float jm = jermax;

  float ts = curacc / jm;
  float te = endacc / jm;

  float vo = curvel - (jm * ts * ts) / 2.0f;

  float vre = vo + jm * (te * te) / 2.0f;
  float srs = vo * ts + jm * (ts * ts * ts) / 6.0f;
  float sre = vo * te + jm * (te * te * te) / 6.0f;

  c1->velini = vo;
  c1->velend = vre;
  c1->disbeg = srs;
  c1->disend = sre;
  c1->accbeg = curacc;
  c1->accend = endacc;
  c1->timbeg = ts;
  c1->timend = te;
  c1->jermax = jm;
}

/* Build linear acceleration period t2 */
static void t2_build(float curvel, float endvel, float accinf,
                     scurve_period_t *c2) {
  float vo = curvel;
  float ve = endvel;
  float as = accinf;

  float tr = (ve - vo) / as;
  float sr = (ve * ve - vo * vo) / (2.0f * as);

  if (vo == ve) {
    tr = 0;
    sr = 0;
  }

  c2->velini = curvel;
  c2->velbeg = curvel;
  c2->velend = endvel;
  c2->disbeg = 0;
  c2->disend = sr;
  c2->accbeg = accinf;
  c2->accend = accinf;
  c2->timbeg = 0;
  c2->timend = tr;
  c2->accinf = accinf;
}

/* Build steady period t4 */
static void t4_build(float curvel, scurve_period_t *c4) {
  c4->velini = curvel;
  c4->velbeg = curvel;
  c4->velend = curvel;
  c4->disbeg = 0;
  c4->disend = 0;
  c4->accbeg = 0;
  c4->accend = 0;
  c4->timbeg = 0;
  c4->timend = 0;
}

/* Calculate delta velocity */
static float delta_vel(float jm, float as) {
  float dvt = 2.0f * as / jm;
  float dv = fabsf((dvt * as) / 2.0f);
  return dv;
}

/* Custom acceleration for short moves */
static void t1_t3_custom_as(float jm, float curvel, float endvel, float *as) {
  float vo = curvel;
  float ve = endvel;
  float vh = (vo + ve) / 2.0f;
  float t1 = sqrtf(2.0f * (vh - vo) / jm);
  *as = jm * t1;
}

/* Build t1,t2,t3 curve - exact from grotius */
static void t1_t2_t3_build(float jermax, float accinf, float curvel,
                           float endvel, scurve_period_t *c1,
                           scurve_period_t *c2, scurve_period_t *c3) {
  float dv = delta_vel(jermax, accinf);
  float velshif = fabsf(curvel - endvel);
  float zeroac = 0;

  if (velshif < dv) {
    /* Short move: using custom acceleration */
    float custas = 0;
    float velhal = (curvel + endvel) * 0.5f;

    t1_t3_custom_as(jermax, curvel, endvel, &custas);

    t1_build(jermax, curvel, zeroac, custas, c1);
    t2_build(velhal, velhal, custas, c2);
    t3_build(jermax, accinf, velhal, custas, zeroac, c3);
  } else {
    /* Full t1,t2,t3 */
    t1_build(jermax, curvel, zeroac, accinf, c1);

    float v = velshif - dv;
    if (jermax < 0) {
      v = -fabsf(v);
    }
    t2_build(c1->velend, c1->velend + v, accinf, c2);
    t3_build(jermax, accinf, c2->velend, accinf, zeroac, c3);
  }
}

/*============================================================================
 * CURVE BUILDERS - Exact from grotius
 *============================================================================*/

/* Forward curve (acceleration) - exact from grotius */
static void forward_curve_build(scurve_state_t *s) {
  float jermax = s->jermax;
  float accinf = s->maxacc * 2.0f;
  float maxvel = s->maxvel;
  float curvel = s->vr;
  float curacc = s->ar;
  float delvel = delvelocity(s);
  float endacc = 0;

  s->oldpos += s->sr;
  s->sr = 0;
  s->curtim = 0;

  zero_period(&s->c0);
  zero_period(&s->c1);
  zero_period(&s->c2);
  zero_period(&s->c3);
  zero_period(&s->c4);

  if (curacc == 0) {
    jermax = fabsf(jermax);
    accinf = fabsf(accinf);
    t1_t2_t3_build(jermax, accinf, curvel, maxvel, &s->c1, &s->c2, &s->c3);
  }
  if (curacc < 0) {
    jermax = -fabsf(jermax);
    accinf = -fabsf(accinf);
    t3_build(jermax, accinf, curvel, curacc, endacc, &s->c0);

    jermax = fabsf(s->jermax);
    accinf = fabsf(s->maxacc * 2.0f);
    t1_t2_t3_build(jermax, accinf, s->c0.velend, maxvel, &s->c1, &s->c2,
                   &s->c3);
  }
  if (curacc > 0) {
    jermax = fabsf(jermax);
    accinf = fabsf(accinf);
    t3_build(jermax, accinf, curvel, curacc, endacc, &s->c3);

    if (s->c3.velend == maxvel) {
      /* use t3 only */
    } else if (curacc == accinf) {
      t2_build(curvel, maxvel - (0.5f * delvel), accinf, &s->c2);
      t3_build(jermax, accinf, s->c2.velend, accinf, endacc, &s->c3);
    } else {
      t1_build(jermax, curvel, curacc, accinf, &s->c1);
      float vo = s->c1.velini;
      t1_t2_t3_build(jermax, accinf, vo, maxvel, &s->c1, &s->c2, &s->c3);
      float as = s->c1.accend;
      t1_build(jermax, curvel, curacc, as, &s->c1);
    }
  }

  t4_build(s->maxvel, &s->c4);
  s->c4.timbeg = 0;
  s->c4.timend = INFINITY;
  s->c4.velbeg = s->maxvel;
  s->c4.disbeg = 0;
  s->c4.disend = INFINITY;
  s->c4.accbeg = 0;
  s->c4.accend = 0;
}

/* Stop curve (deceleration) - exact from grotius */
static void stop_curve_build(scurve_state_t *s) {
  float jermax = s->jermax;
  float accinf = s->maxacc * 2.0f;
  float curvel = s->vr;
  float curacc = s->ar;
  float delvel = delvelocity(s);
  float endacc = s->endacc;
  float endvel = s->endvel;

  s->oldpos += s->sr;
  s->sr = 0;
  s->curtim = 0;

  zero_period(&s->c0);
  zero_period(&s->c1);
  zero_period(&s->c2);
  zero_period(&s->c3);
  zero_period(&s->c4);

  if (curacc == 0) {
    jermax = -fabsf(jermax);
    accinf = -fabsf(accinf);
    t1_t2_t3_build(jermax, accinf, curvel, endvel, &s->c1, &s->c2, &s->c3);
  }

  if (curacc > 0) {
    jermax = fabsf(jermax);
    accinf = fabsf(accinf);
    t3_build(jermax, accinf, curvel, curacc, endacc, &s->c0);

    jermax = -fabsf(s->jermax);
    accinf = -fabsf(s->maxacc * 2.0f);
    t1_t2_t3_build(jermax, accinf, s->c0.velend, endvel, &s->c1, &s->c2,
                   &s->c3);
  }

  if (curacc < 0) {
    jermax = -fabsf(jermax);
    accinf = -fabsf(accinf);
    t3_build(jermax, accinf, curvel, curacc, endacc, &s->c3);
    if (s->c3.velend == endvel) {
      /* use t3 only */
    } else if (curacc == accinf) {
      t2_build(curvel, 0.5f * delvel, accinf, &s->c2);
      t3_build(jermax, accinf, s->c2.velend, accinf, endacc, &s->c3);
    } else {
      t1_build(jermax, curvel, curacc, accinf, &s->c1);
      float vo = s->c1.velini;
      t1_t2_t3_build(jermax, accinf, vo, endvel, &s->c1, &s->c2, &s->c3);
      float as = s->c1.accend;
      t1_build(jermax, curvel, curacc, as, &s->c1);
    }
  }

  t4_build(s->endvel, &s->c4);
  s->c4.timbeg = 0;
  s->c4.timend = INFINITY;
  s->c4.velbeg = s->endvel;
  s->c4.velend = s->endvel;
  s->c4.disbeg = 0;
  s->c4.disend = INFINITY;
  s->c4.accbeg = 0;
  s->c4.accend = 0;
}

/* Calculate stop length - exact from grotius */
static void stop_length(scurve_state_t *s, float *length, float *time) {
  scurve_state_t data = *s;
  stop_curve_build(&data);
  *length = stot_period(data.c0) + stot_period(data.c1) + stot_period(data.c2) +
            stot_period(data.c3);
  *time = ttot_period(data.c0) + ttot_period(data.c1) + ttot_period(data.c2) +
          ttot_period(data.c3);
}

/* jog_update - exact from grotius (called at 20kHz) */
static void jog_update(scurve_state_t *s) {
  s->curtim += s->intval;
  s->oldpos = s->sr;

  float t0 = ttot_period(s->c0);
  float t1 = ttot_period(s->c1);
  float t2 = ttot_period(s->c2);
  float t3 = ttot_period(s->c3);

  if (s->curtim < t0) {
    t3_play(s->curtim, s->c0.accinf, s->c0.jermax, s->c0.timbeg, s->c0.timend,
            s->c0.velini, s->c0.disbeg, &s->vr, &s->sr, &s->ar);
  }
  if (s->curtim >= t0 && s->curtim <= t0 + t1) {
    t1_play(s->curtim - t0, s->c1.accinf, s->c1.jermax, s->c1.timbeg,
            s->c1.timend, s->c1.velini, s->c1.disbeg, &s->vr, &s->sr, &s->ar);
    s->sr += stot_period(s->c0);
  }
  if (s->curtim > t0 + t1 && s->curtim < t0 + t1 + t2) {
    t2_play(s->curtim - (t0 + t1), s->c2.accinf, s->c2.timbeg, s->c2.timend,
            s->c2.velbeg, &s->vr, &s->sr, &s->ar);
    s->sr += stot_period(s->c0) + stot_period(s->c1);
  }
  if (s->curtim >= t0 + t1 + t2 && s->curtim <= t0 + t1 + t2 + t3) {
    t3_play(s->curtim - (t0 + t1 + t2), s->c3.accinf, s->c3.jermax,
            s->c3.timbeg, s->c3.timend, s->c3.velini, s->c3.disbeg, &s->vr,
            &s->sr, &s->ar);
    s->sr += stot_period(s->c0) + stot_period(s->c1) + stot_period(s->c2);
  }
  if (s->curtim > t0 + t1 + t2 + t3) {
    t4_play(s->curtim - (t0 + t1 + t2 + t3), s->c4.timbeg, s->c4.velbeg, &s->vr,
            &s->sr, &s->ar);
    s->sr += stot_period(s->c0) + stot_period(s->c1) + stot_period(s->c2) +
             stot_period(s->c3);
  }

  s->incpos = s->sr - s->oldpos;

  if (s->revers) {
    s->guipos -= s->incpos;
    s->guivel = -fabsf(s->vr);
    s->guiacc = -fabsf(s->ar);
  } else {
    s->guipos += s->incpos;
    s->guivel = s->vr;
    s->guiacc = s->ar;
  }

  if (s->modpos && s->finish) {
    s->guipos = s->tarpos;
  }
}

/* jog_position_fwd - exact from grotius */
static void jog_position_fwd(scurve_state_t *s, int enable, float tarpos) {
  s->tarpos = tarpos;
  s->revers = 0;

  if (!enable) {
    s->pd.stopinit = 0;
  }

  stop_length(s, &s->pd.stopdist, &s->pd.stoptime);
  s->pd.overshoot = (s->guipos + s->pd.stopdist) - tarpos;

  if (s->guivel >= 0 && s->pd.overshoot > 0) {
    if (!s->pd.stopinit) {
      int cycles = (int)(s->pd.stoptime / s->intval);
      if (cycles > 0)
        s->pd.dist_remove_a_cycle = s->pd.overshoot / cycles;
      s->pd.stopinit = 1;
    }
    stop_curve_build(s);
    s->guipos -= s->pd.dist_remove_a_cycle;
    return;
  }

  if (enable) {
    forward_curve_build(s);
  } else {
    s->endvel = 0;
    stop_curve_build(s);
  }
}

/* jog_position_rev - exact from grotius */
static void jog_position_rev(scurve_state_t *s, int enable, float tarpos) {
  s->tarpos = tarpos;
  s->revers = 1;

  if (!enable) {
    s->pd.stopinit = 0;
  }

  stop_length(s, &s->pd.stopdist, &s->pd.stoptime);
  s->pd.stopdist = -fabsf(s->pd.stopdist);

  s->pd.overshoot = (s->guipos + s->pd.stopdist) - s->tarpos;

  if (s->guivel <= 0 && s->pd.overshoot < 0) {
    if (!s->pd.stopinit) {
      int cycles = (int)(s->pd.stoptime / s->intval);
      if (cycles > 0)
        s->pd.dist_remove_a_cycle = s->pd.overshoot / cycles;
      s->pd.stopinit = 1;
    }
    stop_curve_build(s);
    s->guipos += s->pd.dist_remove_a_cycle;
    return;
  }

  if (enable) {
    forward_curve_build(s);
  } else {
    s->endvel = 0;
    stop_curve_build(s);
  }
}

/*============================================================================
 * PUBLIC API
 *============================================================================*/

uint32_t scurve_velocity_to_arr(float velocity) {
  if (velocity <= 0)
    return SCURVE_ARR_MAX;

  uint32_t arr = (uint32_t)(SCURVE_PWM_TIMER_FREQ / velocity);

  if (arr < SCURVE_ARR_MIN)
    arr = SCURVE_ARR_MIN;
  if (arr > SCURVE_ARR_MAX)
    arr = SCURVE_ARR_MAX;

  return arr;
}

void scurve_init(scurve_state_t *s, float jerk_max, float accel_max,
                 float vel_max) {
  zero_period(&s->c0);
  zero_period(&s->c1);
  zero_period(&s->c2);
  zero_period(&s->c3);
  zero_period(&s->c4);

  s->intval = SCURVE_TICK_INTERVAL;
  s->jermax = jerk_max;
  s->maxacc = accel_max;
  s->maxvel = vel_max;

  s->curtim = 0;
  s->guipos = 0;
  s->guivel = 0;
  s->guiacc = 0;
  s->vr = 0;
  s->ar = 0;
  s->sr = 0;
  s->incpos = 0;
  s->oldpos = 0;
  s->tarpos = 0;
  s->endvel = 0;
  s->endacc = 0;

  s->finish = 1;
  s->active = 0;
  s->revers = 0;
  s->modpos = 1;

  s->pd.stopdist = 0;
  s->pd.stoptime = 0;
  s->pd.overshoot = 0;
  s->pd.dist_remove_a_cycle = 0;
  s->pd.stopinit = 0;
  s->pd.btn_fwd = 0;
  s->pd.btn_rev = 0;

  s->arr_value = SCURVE_ARR_MAX;
  s->tick_counter = 0;
}

uint32_t scurve_move_to(scurve_state_t *s, float current_pos,
                        float target_pos) {
  float distance = target_pos - current_pos;

  if (fabsf(distance) < 1.0f) {
    s->finish = 1;
    s->active = 0;
    return 0;
  }

  /* Initialize state */
  s->guipos = current_pos;
  s->tarpos = target_pos;
  s->vr = 0;
  s->ar = 0;
  s->sr = 0;
  s->oldpos = 0;
  s->curtim = 0;
  s->endvel = 0;
  s->endacc = 0;
  s->modpos = 1;
  s->pd.stopinit = 0;

  /* Set direction */
  s->revers = (distance < 0) ? 1 : 0;

  /* Build initial forward curve */
  forward_curve_build(s);

  s->finish = 0;
  s->active = 1;
  s->arr_value = SCURVE_ARR_MAX;
  s->tick_counter = 0;

  return s->arr_value;
}

uint32_t scurve_update(scurve_state_t *s) {
  if (!s->active)
    return 0;

  /* Play the curve first to update velocity */
  jog_update(s);

  /* Increment tick counter */
  s->tick_counter++;

  /* Only check for decel periodically (every N ticks) to reduce CPU load */
  if (!s->pd.stopinit && s->vr > 0.1f &&
      s->tick_counter >= SCURVE_DECEL_CHECK_INTERVAL) {
    s->tick_counter = 0;

    /* Calculate stopping distance from current velocity */
    float stopdist, stoptime;
    stop_length(s, &stopdist, &stoptime);
    stopdist = fabsf(stopdist);

    float remaining = fabsf(s->tarpos - s->guipos);

    /* Start decelerating when remaining distance <= stopping distance */
    if (remaining <= stopdist) {
      int cycles = (int)(stoptime / s->intval);
      if (cycles > 0) {
        float overshoot = stopdist - remaining;
        s->pd.dist_remove_a_cycle = overshoot / cycles;
      }
      s->pd.stopinit = 1;
      stop_curve_build(s); /* Switch to decel curve */
    }
  }

  /* Apply position correction during deceleration */
  if (s->pd.stopinit && s->pd.dist_remove_a_cycle != 0) {
    if (s->revers) {
      s->guipos += s->pd.dist_remove_a_cycle;
    } else {
      s->guipos -= s->pd.dist_remove_a_cycle;
    }
  }

  /* Check if finished - use tight thresholds for smooth finish */
  float remaining = fabsf(s->tarpos - s->guipos);
  if (s->pd.stopinit && s->vr <= 0.01f && remaining < 0.5f) {
    s->finish = 1;
    s->active = 0;
    s->guipos = s->tarpos;
    s->guivel = 0;
    return 0;
  }

  /* Convert velocity to ARR */
  s->arr_value = scurve_velocity_to_arr(fabsf(s->guivel));

  return s->arr_value;
}

void scurve_stop(scurve_state_t *s) {
  if (!s->active)
    return;

  s->endvel = 0;
  s->endacc = 0;
  stop_curve_build(s);
}
