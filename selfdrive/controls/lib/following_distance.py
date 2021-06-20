import cereal.messaging as messaging
from selfdrive.controls.lib.drive_helpers import MPC_COST_LONG
from common.numpy_fast import interp

# 三档跟车距离配置（市区/低速模式）
ONE_BAR_BP = [-0.4, 2.1]
ONE_BAR_V = [0.9, 2.2]
TWO_BAR_BP = [-0.4, 1.9]
TWO_BAR_V = [1.3, 2.2]
THREE_BAR_BP = [0.0, 3.0]
THREE_BAR_V = [1.8, 2.1]


# 三档跟车距离配置（高速模式）
ONE_BAR_H_BP = [0.0, 2.5]
ONE_BAR_H_V = [0.9, 1.3]
TWO_BAR_H_BP = [0.0, 3.0]
TWO_BAR_H_V = [1.3, 1.6]
THREE_BAR_H_BP = [0.0, 4.0]
THREE_BAR_H_V = [1.8, 1.9]

# TR 默认值
TR_DEFAULT = 1.8

# 根据当前速度判断使用 [市区/低速模式] 还是 [高速模式]
# 70km/h ≈ 19.44 m/s
CITY_SPEED = 19.44


class FollowingDistance():
  def __init__(self, libmpc):
    self.libmpc = libmpc
    self.TR = TR_DEFAULT
    self.cost = 0.0
    self.last_cost = 0.0
    self.v_rel = 0.0
    self.is_city_mode = False
    self.tr_btn_val = 4
    self.sm = messaging.SubMaster(['trButton'])

  def _get_TR(self):
    # Adjust distance from lead car when distance button pressed
    if self.tr_btn_val == 1:
      if self.is_city_mode:
        TR = interp(-self.v_rel, ONE_BAR_BP, ONE_BAR_V)
      else:
        TR = interp(-self.v_rel, ONE_BAR_H_BP, ONE_BAR_H_V)

    elif self.tr_btn_val == 2:
      if self.is_city_mode:
        TR = interp(-self.v_rel, TWO_BAR_BP, TWO_BAR_V)
      else:
        TR = interp(-self.v_rel, TWO_BAR_H_BP, TWO_BAR_H_V)

    elif self.tr_btn_val == 3:
      if self.is_city_mode:
        TR = interp(-self.v_rel, THREE_BAR_BP, THREE_BAR_V)
      else:
        TR = interp(-self.v_rel, THREE_BAR_H_BP, THREE_BAR_H_V)

    else:
      TR = TR_DEFAULT

    return TR

  def _get_cost(self):
    TRs = [0.9, 1.8, 2.7]
    # costs = [1.0, 0.115, 0.05]
    costs = [0.5, 0.1, 0.05]
    cost = interp(self.TR, TRs, costs)
    return cost

  def update(self, v_ego, v_lead):
    # Calculate conditions
    # calculate relative velocity vs lead car
    self.v_rel = v_lead - v_ego

    # Is the car running surface city speeds?
    self.is_city_mode = v_ego < CITY_SPEED

    # update ui button value
    self.sm.update(0)
    if self.sm.updated['trButton']:
      self.tr_btn_val = self.sm['trButton'].val

    self.TR = self._get_TR()
    self.cost = self._get_cost()

    # update cost
    if self.cost != self.last_cost:
      self.libmpc.change_tr(MPC_COST_LONG.TTC, self.cost, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
      self.last_cost = self.cost

    return self.TR
