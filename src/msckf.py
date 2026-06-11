import time
import os
import numpy as np
from scipy.stats import chi2
from .utils import *
from .feature import BaseFeature
from .feature import Feature
from collections import namedtuple
from .diagnostics import get_diagnostic_logger

def _make_output_filepath():
    base = os.getenv('OUTPUT_DIR', os.path.join('results', 'txts'))
    os.makedirs(base, exist_ok=True)
    name   = os.getenv('DATASET_NAME', 'unknown')
    offset = os.getenv('TIME_OFFSET',  '0')
    fname  = f"output_{name}_offset{offset}.txt"
    return os.path.join(base, fname)

class IMUState(object):
    # id для следующего состояния IMU
    next_id = 0

    # Вектор гравитации в мировой СК
    gravity = np.array([0., 0., -9.81])

    # Смещение между системой IMU и системой корпуса.
    # Преобразует вектор из системы IMU в систему корпуса.
    # Ось z корпуса должна быть направлена вверх.
    # Обычно это преобразование — тождественное.
    T_imu_body = Isometry3d(np.identity(3), np.zeros(3))

    def __init__(self, new_id=None):
        # Уникальный идентификатор состояния IMU.
        self.id = new_id
        # Время записи состояния.
        self.timestamp = None

        # Ориентация: преобразует вектор из мировой системы в систему IMU (корпуса).
        self.orientation = np.array([0., 0., 0., 1.])

        # Положение системы IMU (корпуса) в мировой системе.
        self.position = np.zeros(3)
        # Скорость системы IMU (корпуса) в мировой системе.
        self.velocity = np.zeros(3)

        # Смещения гироскопа и акселерометра.
        self.gyro_bias = np.zeros(3)
        self.acc_bias = np.zeros(3)

        # Эти три переменные должны физически соответствовать orientation, position и velocity.
        # Используются для корректировки переходных матриц и обеспечения правильного ядра наблюдаемости.
        # Ориентация, положение и скорость для "нулевого" состояния.
        self.orientation_null = np.array([0., 0., 0., 1.])
        self.position_null = np.zeros(3)
        self.velocity_null = np.zeros(3)

        # Преобразование между IMU и левой камерой (cam0).
        self.R_imu_cam0 = np.identity(3)
        self.t_cam0_imu = np.zeros(3)


class CAMState(object):
    # Преобразует вектор из системы cam0 в систему cam1.
    R_cam0_cam1 = None
    t_cam0_cam1 = None

    def __init__(self, new_id=None):
        self.id = new_id
        self.timestamp = None

        # Ориентация: преобразует вектор из мировой системы в систему камеры.
        self.orientation = np.array([0., 0., 0., 1.])

        # Положение системы камеры в мировой системе.
        self.position = np.zeros(3)

        self.orientation_null = np.array([0., 0., 0., 1.])
        self.position_null = np.zeros(3)

        
class StateServer(object):
    """
    
    Хранит одно состояние IMU и несколько состояний камеры для построения измерительной модели.
    
    """
    def __init__(self):
        self.imu_state = IMUState()
        self.cam_states = dict() 

        self.state_cov = np.zeros((21, 21))
        self.continuous_noise_cov = np.zeros((12, 12))



class MSCKF(object):
    def __init__(self, config):
        self.config = config
        self.optimization_config = config.optimization_config

        # Буфер данных IMU.
        # Используется для компенсации несинхронизации или задержек передачи между сообщениями IMU и изображениями.
        self.imu_msg_buffer = []

        # Вектор состояния
        self.state_server = StateServer()
        # Используемые признаки
        self.map_server = dict()   # <FeatureID, Feature>

        # Таблица критических значений chi-квадрат.
        # Инициализируется для доверительного уровня 0.95.
        self.chi_squared_test_table = dict()
        for i in range(1, 200):
            self.chi_squared_test_table[i] = chi2.ppf(0.95, i)

        # Устанавливает начальное состояние IMU.
        # Начальная ориентация и положение задаются равными нулю по умолчанию.
        # Начальную скорость и смещения можно задать параметрами.
        # TODO: имеет ли смысл инициализировать смещения нулём?

        self.state_server.imu_state.velocity = config.velocity
        self.reset_state_cov()

        continuous_noise_cov = np.identity(12)
        continuous_noise_cov[:3, :3] *= self.config.gyro_noise
        continuous_noise_cov[3:6, 3:6] *= self.config.gyro_bias_noise
        continuous_noise_cov[6:9, 6:9] *= self.config.acc_noise
        continuous_noise_cov[9:, 9:] *= self.config.acc_bias_noise
        self.state_server.continuous_noise_cov = continuous_noise_cov

        IMUState.gravity = config.gravity

        T_cam0_imu = np.linalg.inv(config.T_imu_cam0)
        self.state_server.imu_state.R_imu_cam0 = T_cam0_imu[:3, :3].T
        self.state_server.imu_state.t_cam0_imu = T_cam0_imu[:3, 3]

        T_cam0_cam1 = config.T_cn_cnm1
        CAMState.R_cam0_cam1 = T_cam0_cam1[:3, :3]
        CAMState.t_cam0_cam1 = T_cam0_cam1[:3, 3]
        BaseFeature.R_cam0_cam1 = CAMState.R_cam0_cam1
        BaseFeature.t_cam0_cam1 = CAMState.t_cam0_cam1
        IMUState.T_imu_body = Isometry3d(
            config.T_imu_body[:3, :3],
            config.T_imu_body[:3, 3])

        self.tracking_rate = None

        self.is_gravity_set = False

        self.is_first_img = True
        self.diagnostic_logger = get_diagnostic_logger()
        self._diag_timestamp = None
        self._diag_context = "none"
        self._outfile = _make_output_filepath()
        self._prepare_output_file()

    def _prepare_output_file(self):
        append_output = os.getenv('APPEND_OUTPUT', '0').lower() in ('1', 'true', 'yes')
        if append_output and os.path.exists(self._outfile):
            return
        with open(self._outfile, 'w') as f:
            f.write('# timestamp p_x p_y p_z q_x q_y q_z q_w\n')

    def _write_state(self, imu_state):
        line = (
            f"{imu_state.timestamp:.6f} "
            f"{imu_state.position[0]:.9f} {imu_state.position[1]:.9f} {imu_state.position[2]:.9f} "
            f"{imu_state.orientation[0]:.9f} {imu_state.orientation[1]:.9f} "
            f"{imu_state.orientation[2]:.9f} {imu_state.orientation[3]:.9f}\n"
        )
        with open(self._outfile, 'a') as f:
            f.write(line)

    def imu_callback(self, imu_msg):
        """
        Колбэк для обработки сообщений IMU.
        """
        # Сообщения IMU помещаются в буфер, а не обрабатываются сразу.
        # Обработка IMU выполняется при поступлении следующего изображения,
        # что облегчает учёт задержек передачи.

        self.imu_msg_buffer.append(imu_msg)

        if not self.is_gravity_set:
            if len(self.imu_msg_buffer) >= 200:
                self.initialize_gravity_and_bias()
                self.is_gravity_set = True

    def feature_callback(self, feature_msg):
        """
        Колбэк для обработки измерений признаков.
        """

        if not self.is_gravity_set:
            return
        self._diag_timestamp = feature_msg.timestamp
        start = time.time()

        # Запуск системы при получении первого изображения.
        # Кадр с первым изображением принимается за начало координат.
        if self.is_first_img:
            self.is_first_img = False
            self.state_server.imu_state.timestamp = feature_msg.timestamp

        t = time.time()

        # Прогноз состояния IMU.
        # Применяется ко всем сообщениям, полученным до изображения.
        self.batch_imu_processing(feature_msg.timestamp)

        print('---batch_imu_processing    ', time.time() - t)
        t = time.time()

        # Дополняет (расширяет) вектор состояния.
        self.state_augmentation(feature_msg.timestamp)

        print('---state_augmentation      ', time.time() - t)
        t = time.time()

        # Добавляет новые наблюдения к существующим признакам или новым признакам в map server.
        self.add_feature_observations(feature_msg)

        if self.diagnostic_logger is not None:
            imu_state = self.state_server.imu_state
            self.diagnostic_logger.log_msckf_frame({
                "timestamp": feature_msg.timestamp,
                "feature_count": int(len(feature_msg.features)),
                "tracking_rate": float(self.tracking_rate if self.tracking_rate is not None else -1.0),
                "map_features": int(len(self.map_server)),
                "cam_states": int(len(self.state_server.cam_states)),
                "imu_buffer": int(len(self.imu_msg_buffer)),
                "position_x": float(imu_state.position[0]),
                "position_y": float(imu_state.position[1]),
                "position_z": float(imu_state.position[2]),
                "velocity_norm": float(np.linalg.norm(imu_state.velocity)),
            })

        print('---add_feature_observations', time.time() - t)
        t = time.time()

        # Выполняет обновление по измерениям при необходимости.
        # Очищает признаки и состояния камеры.
        self._diag_context = "remove_lost_features"
        self.remove_lost_features()
        self._diag_context = "none"

        print('---remove_lost_features    ', time.time() - t)
        t = time.time()

        self._diag_context = "prune_cam_state_buffer"
        self.prune_cam_state_buffer()
        self._diag_context = "none"

        print('---prune_cam_state_buffer  ', time.time() - t)
        print('---msckf elapsed:          ', time.time() - start, f'({feature_msg.timestamp})')

        try:
            return self.publish(feature_msg.timestamp)
        finally:
            self.online_reset()

    def initialize_gravity_and_bias(self):
        """
        Инициализирует смещения и начальную ориентацию IMU по первым измерениям IMU.
        """
        sum_angular_vel = np.zeros(3)
        sum_linear_acc = np.zeros(3)
        for msg in self.imu_msg_buffer:
            sum_angular_vel += msg.angular_velocity
            sum_linear_acc += msg.linear_acceleration

        gyro_bias = sum_angular_vel / len(self.imu_msg_buffer)
        self.state_server.imu_state.gyro_bias = gyro_bias

        gravity_imu = sum_linear_acc / len(self.imu_msg_buffer)

        gravity_norm = np.linalg.norm(gravity_imu)
        IMUState.gravity = np.array([0., 0., -gravity_norm])

        self.state_server.imu_state.orientation = from_two_vectors(
            -IMUState.gravity, gravity_imu)

    def batch_imu_processing(self, time_bound):
        """
        Propogate the state
        """
        used_imu_msg_count = 0
        for msg in self.imu_msg_buffer:
            imu_time = msg.timestamp
            if imu_time < self.state_server.imu_state.timestamp:
                used_imu_msg_count += 1
                continue
            if imu_time > time_bound:
                break

            self.process_model(
                imu_time, msg.angular_velocity, msg.linear_acceleration)
            used_imu_msg_count += 1

            self.state_server.imu_state.timestamp = imu_time

        self.state_server.imu_state.id = IMUState.next_id
        IMUState.next_id += 1

        self.imu_msg_buffer = self.imu_msg_buffer[used_imu_msg_count:]

    def process_model(self, time, m_gyro, m_acc):
        imu_state = self.state_server.imu_state
        dt = time - imu_state.timestamp

        gyro = m_gyro - imu_state.gyro_bias
        acc = m_acc - imu_state.acc_bias

        F = np.zeros((21, 21))
        G = np.zeros((21, 12))

        R_w_i = to_rotation(imu_state.orientation)

        F[:3, :3] = -skew(gyro)
        F[:3, 3:6] = -np.identity(3)
        F[6:9, :3] = -R_w_i.T @ skew(acc)
        F[6:9, 9:12] = -R_w_i.T
        F[12:15, 6:9] = np.identity(3)

        G[:3, :3] = -np.identity(3)
        G[3:6, 3:6] = np.identity(3)
        G[6:9, 6:9] = -R_w_i.T
        G[9:12, 9:12] = np.identity(3)

        # Аппроксимация матричной экспоненты до третьего порядка.
        # Достаточно точно при dt ≲ 0.01 с.
        Fdt = F * dt
        Fdt_square = Fdt @ Fdt
        Fdt_cube = Fdt_square @ Fdt
        Phi = np.identity(21) + Fdt + Fdt_square/2. + Fdt_cube/6.

        self.predict_new_state(dt, gyro, acc)

        R_kk_1 = to_rotation(imu_state.orientation_null)
        Phi[:3, :3] = to_rotation(imu_state.orientation) @ R_kk_1.T

        u = R_kk_1 @ IMUState.gravity
        # s = (u.T @ u).inverse() @ u.T
        # s = np.linalg.inv(u[:, None] * u) @ u
        s = u / (u @ u)

        A1 = Phi[6:9, :3]
        w1 = skew(imu_state.velocity_null - imu_state.velocity) @ IMUState.gravity
        Phi[6:9, :3] = A1 - (A1 @ u - w1)[:, None] * s

        A2 = Phi[12:15, :3]
        w2 = skew(dt*imu_state.velocity_null+imu_state.position_null - 
            imu_state.position) @ IMUState.gravity
        Phi[12:15, :3] = A2 - (A2 @ u - w2)[:, None] * s

        Q = Phi @ G @ self.state_server.continuous_noise_cov @ G.T @ Phi.T * dt
        self.state_server.state_cov[:21, :21] = (
            Phi @ self.state_server.state_cov[:21, :21] @ Phi.T + Q)

        if len(self.state_server.cam_states) > 0:
            self.state_server.state_cov[:21, 21:] = (
                Phi @ self.state_server.state_cov[:21, 21:])
            self.state_server.state_cov[21:, :21] = (
                self.state_server.state_cov[21:, :21] @ Phi.T)

        self.state_server.state_cov = (
            self.state_server.state_cov + self.state_server.state_cov.T) / 2.

        self.state_server.imu_state.orientation_null = imu_state.orientation
        self.state_server.imu_state.position_null = imu_state.position
        self.state_server.imu_state.velocity_null = imu_state.velocity

    def predict_new_state(self, dt, gyro, acc):
        # TODO: Улучшит ли точность прямое интегрирование с использованием обратного кватерниона?
        gyro_norm = np.linalg.norm(gyro)
        Omega = np.zeros((4, 4))
        Omega[:3, :3] = -skew(gyro)
        Omega[:3, 3] = gyro
        Omega[3, :3] = -gyro

        q = self.state_server.imu_state.orientation
        v = self.state_server.imu_state.velocity
        p = self.state_server.imu_state.position

        if gyro_norm > 1e-5:
            dq_dt = (np.cos(gyro_norm*dt*0.5) * np.identity(4) + 
                np.sin(gyro_norm*dt*0.5)/gyro_norm * Omega) @ q
            dq_dt2 = (np.cos(gyro_norm*dt*0.25) * np.identity(4) + 
                np.sin(gyro_norm*dt*0.25)/gyro_norm * Omega) @ q
        else:
            dq_dt = np.cos(gyro_norm*dt*0.5) * (np.identity(4) + 
                Omega*dt*0.5) @ q
            dq_dt2 = np.cos(gyro_norm*dt*0.25) * (np.identity(4) + 
                Omega*dt*0.25) @ q

        dR_dt_transpose = to_rotation(dq_dt).T
        dR_dt2_transpose = to_rotation(dq_dt2).T

        k1_p_dot = v
        k1_v_dot = to_rotation(q).T @ acc + IMUState.gravity

        k1_v = v + k1_v_dot*dt/2.
        k2_p_dot = k1_v
        k2_v_dot = dR_dt2_transpose @ acc + IMUState.gravity

        k2_v = v + k2_v_dot*dt/2
        k3_p_dot = k2_v
        k3_v_dot = dR_dt2_transpose @ acc + IMUState.gravity

        k3_v = v + k3_v_dot*dt
        k4_p_dot = k3_v
        k4_v_dot = dR_dt_transpose @ acc + IMUState.gravity

        q = dq_dt / np.linalg.norm(dq_dt)
        v = v + (k1_v_dot + 2*k2_v_dot + 2*k3_v_dot + k4_v_dot)*dt/6.
        p = p + (k1_p_dot + 2*k2_p_dot + 2*k3_p_dot + k4_p_dot)*dt/6.

        self.state_server.imu_state.orientation = q
        self.state_server.imu_state.velocity = v
        self.state_server.imu_state.position = p

    def state_augmentation(self, time):
        imu_state = self.state_server.imu_state
        R_i_c = imu_state.R_imu_cam0
        t_c_i = imu_state.t_cam0_imu

        R_w_i = to_rotation(imu_state.orientation)
        R_w_c = R_i_c @ R_w_i
        t_c_w = imu_state.position + R_w_i.T @ t_c_i

        cam_state = CAMState(imu_state.id)
        cam_state.timestamp = time
        cam_state.orientation = to_quaternion(R_w_c)
        cam_state.position = t_c_w
        cam_state.orientation_null = cam_state.orientation
        cam_state.position_null = cam_state.position
        self.state_server.cam_states[imu_state.id] = cam_state

        J = np.zeros((6, 21))
        J[:3, :3] = R_i_c
        J[:3, 15:18] = np.identity(3)
        J[3:6, :3] = skew(R_w_i.T @ t_c_i)
        J[3:6, 12:15] = np.identity(3)
        J[3:6, 18:21] = np.identity(3)

        # old_rows, old_cols = self.state_server.state_cov.shape
        old_size = self.state_server.state_cov.shape[0]   # symmetric
        state_cov = np.zeros((old_size+6, old_size+6))
        state_cov[:old_size, :old_size] = self.state_server.state_cov

        state_cov[old_size:, :old_size] = J @ state_cov[:21, :old_size]
        state_cov[:old_size, old_size:] = state_cov[old_size:, :old_size].T
        state_cov[old_size:, old_size:] = J @ state_cov[:21, :21] @ J.T

        self.state_server.state_cov = (state_cov + state_cov.T) / 2.

    def add_feature_observations(self, feature_msg):
        state_id = self.state_server.imu_state.id
        curr_feature_num = len(self.map_server)
        tracked_feature_num = 0

        for feature in feature_msg.features:
            if feature.id not in self.map_server:
                map_feature = Feature(feature.id, self.optimization_config)
                map_feature.observations[state_id] = np.array([
                    feature.u0, feature.v0, feature.u1, feature.v1])
                self.map_server[feature.id] = map_feature
            else:
                self.map_server[feature.id].observations[state_id] = np.array([
                    feature.u0, feature.v0, feature.u1, feature.v1])
                tracked_feature_num += 1

        self.tracking_rate = tracked_feature_num / (curr_feature_num+1e-5)

    def measurement_jacobian(self, cam_state_id, feature_id):
        """

        Вычисляет якобиан измерения для одного признака, наблюдаемого в одном кадре.

        """

        cam_state = self.state_server.cam_states[cam_state_id]
        feature = self.map_server[feature_id]

        R_w_c0 = to_rotation(cam_state.orientation)
        t_c0_w = cam_state.position

        R_w_c1 = CAMState.R_cam0_cam1 @ R_w_c0
        t_c1_w = t_c0_w - R_w_c1.T @ CAMState.t_cam0_cam1

        # Положение 3D-признака в мировой системе координат
        # и его наблюдение стереокамерами.
        p_w = feature.position
        z = feature.observations[cam_state_id]

        # Преобразует положение признака из мировой системы в системы cam0 и cam1.
        p_c0 = R_w_c0 @ (p_w - t_c0_w)
        p_c1 = R_w_c1 @ (p_w - t_c1_w)
        min_depth = 1e-6
        if (not np.all(np.isfinite(p_c0)) or
            not np.all(np.isfinite(p_c1)) or
            not np.all(np.isfinite(z)) or
            p_c0[2] <= min_depth or
            p_c1[2] <= min_depth):
            # Invalid depth would create NaN/Inf residuals and corrupt the filter.
            return None, None, None

        dz_dpc0 = np.zeros((4, 3))
        dz_dpc0[0, 0] = 1 / p_c0[2]
        dz_dpc0[1, 1] = 1 / p_c0[2]
        dz_dpc0[0, 2] = -p_c0[0] / (p_c0[2] * p_c0[2])
        dz_dpc0[1, 2] = -p_c0[1] / (p_c0[2] * p_c0[2])

        dz_dpc1 = np.zeros((4, 3))
        dz_dpc1[2, 0] = 1 / p_c1[2]
        dz_dpc1[3, 1] = 1 / p_c1[2]
        dz_dpc1[2, 2] = -p_c1[0] / (p_c1[2] * p_c1[2])
        dz_dpc1[3, 2] = -p_c1[1] / (p_c1[2] * p_c1[2])

        dpc0_dxc = np.zeros((3, 6))
        dpc0_dxc[:, :3] = skew(p_c0)
        dpc0_dxc[:, 3:] = -R_w_c0

        dpc1_dxc = np.zeros((3, 6))
        dpc1_dxc[:, :3] = CAMState.R_cam0_cam1 @ skew(p_c0)
        dpc1_dxc[:, 3:] = -R_w_c1

        dpc0_dpg = R_w_c0
        dpc1_dpg = R_w_c1

        H_x = dz_dpc0 @ dpc0_dxc + dz_dpc1 @ dpc1_dxc   # shape: (4, 6)
        H_f = dz_dpc0 @ dpc0_dpg + dz_dpc1 @ dpc1_dpg   # shape: (4, 3)

        A = H_x   # shape: (4, 6)
        u = np.zeros(6)
        u[:3] = to_rotation(cam_state.orientation_null) @ IMUState.gravity
        u[3:] = skew(p_w - cam_state.position_null) @ IMUState.gravity

        H_x = A - (A @ u)[:, None] * u / (u @ u)
        H_f = -H_x[:4, 3:6]

        r = z - np.array([*p_c0[:2]/p_c0[2], *p_c1[:2]/p_c1[2]])

        # H_x: shape (4, 6)
        # H_f: shape (4, 3)
        # r  : shape (4,)
        return H_x, H_f, r

    def feature_jacobian(self, feature_id, cam_state_ids):
        """
        Вычисляет якобиан всех наблюдений данного признака по всем заданным состояниям камеры.
        """
        feature = self.map_server[feature_id]

        # Проверяет, в скольких состояниях камеры (по заданным id) этот признак действительно наблюдался.

        valid_cam_state_ids = []
        for cam_id in cam_state_ids:
            if cam_id in feature.observations:
                valid_cam_state_ids.append(cam_id)

        jacobian_row_size = 4 * len(valid_cam_state_ids)

        cam_states = self.state_server.cam_states
        H_xj = np.zeros((jacobian_row_size, 
            21+len(self.state_server.cam_states)*6))
        H_fj = np.zeros((jacobian_row_size, 3))
        r_j = np.zeros(jacobian_row_size)

        stack_count = 0
        for cam_id in valid_cam_state_ids:
            H_xi, H_fi, r_i = self.measurement_jacobian(cam_id, feature.id)
            if H_xi is None:
                continue

            idx = list(self.state_server.cam_states.keys()).index(cam_id)
            H_xj[stack_count:stack_count+4, 21+6*idx:21+6*(idx+1)] = H_xi
            H_fj[stack_count:stack_count+4, :3] = H_fi
            r_j[stack_count:stack_count+4] = r_i
            stack_count += 4

        H_xj = H_xj[:stack_count]
        H_fj = H_fj[:stack_count]
        r_j = r_j[:stack_count]
        if stack_count <= 3 or not np.all(np.isfinite(H_xj)) or not np.all(np.isfinite(r_j)):
            return np.empty((0, H_xj.shape[1])), np.empty(0)

        U, _, _ = np.linalg.svd(H_fj)
        A = U[:, 3:]

        H_x = A.T @ H_xj
        r = A.T @ r_j
        if not np.all(np.isfinite(H_x)) or not np.all(np.isfinite(r)):
            return np.empty((0, H_xj.shape[1])), np.empty(0)

        return H_x, r

    def measurement_update(self, H, r):
        if len(H) == 0 or len(r) == 0:
            if self.diagnostic_logger is not None:
                self.diagnostic_logger.log_msckf_update({
                    "timestamp": self._diag_timestamp,
                    "context": self._diag_context,
                    "status": "empty",
                    "H_rows": int(getattr(H, "shape", [0, 0])[0]) if hasattr(H, "shape") else 0,
                    "H_cols": int(getattr(H, "shape", [0, 0])[1]) if hasattr(H, "shape") and len(H.shape) > 1 else 0,
                    "r_len": int(len(r)) if hasattr(r, "__len__") else 0,
                    "r_norm": 0.0,
                    "delta_norm": 0.0,
                    "delta_orientation_norm": 0.0,
                    "delta_velocity_norm": 0.0,
                    "delta_position_norm": 0.0,
                    "delta_gyro_bias_norm": 0.0,
                    "delta_acc_bias_norm": 0.0,
                    "num_cam_states": int(len(self.state_server.cam_states)),
                    "map_features": int(len(self.map_server)),
                })
            return
        if not np.all(np.isfinite(H)) or not np.all(np.isfinite(r)):
            if self.diagnostic_logger is not None:
                self.diagnostic_logger.log_msckf_update({
                    "timestamp": self._diag_timestamp,
                    "context": self._diag_context,
                    "status": "nonfinite_input",
                    "H_rows": int(H.shape[0]),
                    "H_cols": int(H.shape[1]) if len(H.shape) > 1 else 0,
                    "r_len": int(len(r)),
                    "r_norm": float(np.linalg.norm(r)) if np.all(np.isfinite(r)) else -1.0,
                    "delta_norm": 0.0,
                    "delta_orientation_norm": 0.0,
                    "delta_velocity_norm": 0.0,
                    "delta_position_norm": 0.0,
                    "delta_gyro_bias_norm": 0.0,
                    "delta_acc_bias_norm": 0.0,
                    "num_cam_states": int(len(self.state_server.cam_states)),
                    "map_features": int(len(self.map_server)),
                })
            return

        # Декомпозирует итоговую матрицу якобиана для снижения вычислительной сложности

        if H.shape[0] > H.shape[1]:
            Q, R = np.linalg.qr(H, mode='reduced')  # if M > N, return (M, N), (N, N)
            H_thin = R         # shape (N, N)
            r_thin = Q.T @ r   # shape (N,)
        else:
            H_thin = H   # shape (M, N)
            r_thin = r   # shape (M)

        P = self.state_server.state_cov
        S = H_thin @ P @ H_thin.T + (self.config.observation_noise * 
            np.identity(len(H_thin)))

        def _diag_matrix_stats(matrix):
            try:
                svals = np.linalg.svd(matrix, compute_uv=False)
                if len(svals) == 0:
                    return 0, 0.0, 0.0, -1.0

                smax = float(svals[0])
                smin = float(svals[-1])
                tol = np.finfo(float).eps * max(matrix.shape) * smax
                rank = int(np.sum(svals > tol))
                cond = float(smax / smin) if smin > 0.0 else float("inf")
                return rank, smin, smax, cond
            except Exception:
                return -1, -1.0, -1.0, -1.0

        H_thin_rank, H_thin_smin, H_thin_smax, H_thin_cond = _diag_matrix_stats(H_thin)
        S_rank, S_smin, S_smax, S_cond = _diag_matrix_stats(S)
        r_thin_norm = float(np.linalg.norm(r_thin))
        try:
            K_transpose = np.linalg.solve(S, H_thin @ P)
        except np.linalg.LinAlgError:
            if self.diagnostic_logger is not None:
                self.diagnostic_logger.log_msckf_update({
                    "timestamp": self._diag_timestamp,
                    "context": self._diag_context,
                    "status": "singular_S",
                    "H_rows": int(H.shape[0]),
                    "H_cols": int(H.shape[1]) if len(H.shape) > 1 else 0,
                    "r_len": int(len(r)),
                    "r_norm": float(np.linalg.norm(r)),
                    "delta_norm": 0.0,
                    "delta_orientation_norm": 0.0,
                    "delta_velocity_norm": 0.0,
                    "delta_position_norm": 0.0,
                    "delta_gyro_bias_norm": 0.0,
                    "delta_acc_bias_norm": 0.0,
                    "num_cam_states": int(len(self.state_server.cam_states)),
                    "map_features": int(len(self.map_server)),
                })
            return
        K = K_transpose.T   # shape (N, K)

        delta_x = K @ r_thin
        if not np.all(np.isfinite(delta_x)):
            if self.diagnostic_logger is not None:
                self.diagnostic_logger.log_msckf_update({
                    "timestamp": self._diag_timestamp,
                    "context": self._diag_context,
                    "status": "nonfinite_delta",
                    "H_rows": int(H.shape[0]),
                    "H_cols": int(H.shape[1]) if len(H.shape) > 1 else 0,
                    "r_len": int(len(r)),
                    "r_norm": float(np.linalg.norm(r)),
                    "delta_norm": -1.0,
                    "delta_orientation_norm": -1.0,
                    "delta_velocity_norm": -1.0,
                    "delta_position_norm": -1.0,
                    "delta_gyro_bias_norm": -1.0,
                    "delta_acc_bias_norm": -1.0,
                    "num_cam_states": int(len(self.state_server.cam_states)),
                    "map_features": int(len(self.map_server)),
                })
            return

        delta_x_imu = delta_x[:21]

        if os.getenv("VIO_DIAGNOSTICS") == "1":
            try:
                import csv
                from pathlib import Path

                diag_dir = os.getenv("VIO_DIAGNOSTICS_DIR")
                if diag_dir:
                    cond_path = Path(diag_dir) / "msckf_conditioning.csv"
                    cond_path.parent.mkdir(parents=True, exist_ok=True)

                    r_norm = float(np.linalg.norm(r))
                    delta_norm = float(np.linalg.norm(delta_x))
                    delta_pos_norm = float(np.linalg.norm(delta_x_imu[12:15]))
                    delta_vel_norm = float(np.linalg.norm(delta_x_imu[6:9]))
                    eps = 1e-12

                    row = {
                        "timestamp": self._diag_timestamp,
                        "context": self._diag_context,
                        "status": "applied",
                        "H_rows": int(H.shape[0]),
                        "H_cols": int(H.shape[1]) if len(H.shape) > 1 else 0,
                        "H_thin_rows": int(H_thin.shape[0]),
                        "H_thin_cols": int(H_thin.shape[1]) if len(H_thin.shape) > 1 else 0,
                        "r_norm": r_norm,
                        "r_thin_norm": r_thin_norm,
                        "H_thin_rank": H_thin_rank,
                        "H_thin_smin": H_thin_smin,
                        "H_thin_smax": H_thin_smax,
                        "H_thin_cond": H_thin_cond,
                        "S_rank": S_rank,
                        "S_smin": S_smin,
                        "S_smax": S_smax,
                        "S_cond": S_cond,
                        "K_norm": float(np.linalg.norm(K)),
                        "delta_norm": delta_norm,
                        "delta_position_norm": delta_pos_norm,
                        "delta_velocity_norm": delta_vel_norm,
                        "delta_over_r": delta_norm / (r_norm + eps),
                        "delta_pos_over_r": delta_pos_norm / (r_norm + eps),
                        "delta_pos_over_rthin": delta_pos_norm / (r_thin_norm + eps),
                        "num_cam_states": int(len(self.state_server.cam_states)),
                        "map_features": int(len(self.map_server)),
                    }

                    fields = list(row.keys())
                    write_header = not cond_path.exists()
                    with cond_path.open("a", newline="") as handle:
                        writer = csv.DictWriter(handle, fieldnames=fields)
                        if write_header:
                            writer.writeheader()
                        writer.writerow(row)

            except Exception as exc:
                print(f"[diagnostics] failed to write msckf_conditioning.csv: {exc}")
        if self.diagnostic_logger is not None:
            self.diagnostic_logger.log_msckf_update({
                "timestamp": self._diag_timestamp,
                "context": self._diag_context,
                "status": "applied",
                "H_rows": int(H.shape[0]),
                "H_cols": int(H.shape[1]) if len(H.shape) > 1 else 0,
                "r_len": int(len(r)),
                "r_norm": float(np.linalg.norm(r)),
                "delta_norm": float(np.linalg.norm(delta_x)),
                "delta_orientation_norm": float(np.linalg.norm(delta_x_imu[:3])),
                "delta_velocity_norm": float(np.linalg.norm(delta_x_imu[6:9])),
                "delta_position_norm": float(np.linalg.norm(delta_x_imu[12:15])),
                "delta_gyro_bias_norm": float(np.linalg.norm(delta_x_imu[3:6])),
                "delta_acc_bias_norm": float(np.linalg.norm(delta_x_imu[9:12])),
                "num_cam_states": int(len(self.state_server.cam_states)),
                "map_features": int(len(self.map_server)),
            })

        if (np.linalg.norm(delta_x_imu[6:9]) > 0.5 or 
            np.linalg.norm(delta_x_imu[12:15]) > 1.0):
            print('[Warning] Update change is too large')

        dq_imu = small_angle_quaternion(delta_x_imu[:3])
        imu_state = self.state_server.imu_state
        imu_state.orientation = quaternion_multiplication(
            dq_imu, imu_state.orientation)
        imu_state.gyro_bias += delta_x_imu[3:6]
        imu_state.velocity += delta_x_imu[6:9]
        imu_state.acc_bias += delta_x_imu[9:12]
        imu_state.position += delta_x_imu[12:15]

        dq_extrinsic = small_angle_quaternion(delta_x_imu[15:18])
        imu_state.R_imu_cam0 = to_rotation(dq_extrinsic) @ imu_state.R_imu_cam0
        imu_state.t_cam0_imu += delta_x_imu[18:21]

        for i, (cam_id, cam_state) in enumerate(
                self.state_server.cam_states.items()):
            delta_x_cam = delta_x[21+i*6:27+i*6]
            dq_cam = small_angle_quaternion(delta_x_cam[:3])
            cam_state.orientation = quaternion_multiplication(
                dq_cam, cam_state.orientation)
            cam_state.position += delta_x_cam[3:]

        I_KH = np.identity(len(K)) - K @ H_thin
        R_obs = self.config.observation_noise * np.identity(len(H_thin))
        state_cov = (
            I_KH @ self.state_server.state_cov @ I_KH.T +
            K @ R_obs @ K.T
        )

        self.state_server.state_cov = (state_cov + state_cov.T) / 2.

    def gating_test(self, H, r):
        if len(H) == 0 or len(r) == 0:
            return False
        if not np.all(np.isfinite(H)) or not np.all(np.isfinite(r)):
            return False

        dof = len(r)
        P1 = H @ self.state_server.state_cov @ H.T
        P2 = self.config.observation_noise * np.identity(len(H))
        try:
            gamma = r @ np.linalg.solve(P1+P2, r)
        except np.linalg.LinAlgError:
            return False
        if not np.isfinite(gamma):
            return False

        threshold = self.chi_squared_test_table.get(dof, chi2.ppf(0.95, dof))

        accepted = bool(gamma < threshold)

        if self.diagnostic_logger is not None:
            self.diagnostic_logger.log_msckf_gating({
                "timestamp": self._diag_timestamp,
                "context": self._diag_context,
                "dof": int(dof),
                "gamma": float(gamma),
                "threshold": float(threshold),
                "accepted": int(accepted),
                "r_norm": float(np.linalg.norm(r)),
                "H_rows": int(H.shape[0]),
                "H_cols": int(H.shape[1]) if len(H.shape) > 1 else 0,
            })

        if accepted:
            return True
        else:
            return False


    def _feature_geometry_stats(self, feature, cam_state_ids):
        stats = {
            "feature_id": int(feature.id),
            "track_len": int(len(feature.observations)),
            "used_len": int(len(cam_state_ids)),
            "is_initialized": int(bool(feature.is_initialized)),
            "position_norm": float("nan"),
            "depth0_min": float("nan"),
            "depth0_median": float("nan"),
            "depth0_max": float("nan"),
            "all_baseline": float("nan"),
            "used_baseline": float("nan"),
            "all_parallax_deg": float("nan"),
            "used_parallax_deg": float("nan"),
            "stereo_disparity_min": float("nan"),
            "stereo_disparity_median": float("nan"),
            "stereo_disparity_max": float("nan"),
        }

        if np.all(np.isfinite(feature.position)):
            stats["position_norm"] = float(np.linalg.norm(feature.position))

        all_ids = [cid for cid in feature.observations.keys()
                   if cid in self.state_server.cam_states]
        used_ids = [cid for cid in cam_state_ids
                    if cid in feature.observations and cid in self.state_server.cam_states]
        stats["used_len"] = int(len(used_ids))

        depths = []
        for cid in all_ids:
            cam_state = self.state_server.cam_states[cid]
            R_w_c0 = to_rotation(cam_state.orientation)
            p_c0 = R_w_c0 @ (feature.position - cam_state.position)
            if np.all(np.isfinite(p_c0)):
                depths.append(float(p_c0[2]))

        if len(depths) > 0:
            arr = np.asarray(depths, dtype=float)
            arr = arr[np.isfinite(arr)]
            if len(arr) > 0:
                stats["depth0_min"] = float(np.min(arr))
                stats["depth0_median"] = float(np.median(arr))
                stats["depth0_max"] = float(np.max(arr))

        disparities = []
        for cid in all_ids:
            m = np.asarray(feature.observations[cid], dtype=float)
            if len(m) >= 4 and np.all(np.isfinite(m[:4])):
                disparities.append(abs(float(m[0] - m[2])))

        if len(disparities) > 0:
            arr = np.asarray(disparities, dtype=float)
            arr = arr[np.isfinite(arr)]
            if len(arr) > 0:
                stats["stereo_disparity_min"] = float(np.min(arr))
                stats["stereo_disparity_median"] = float(np.median(arr))
                stats["stereo_disparity_max"] = float(np.max(arr))

        def bearing_world(cid):
            cam_state = self.state_server.cam_states[cid]
            m = np.asarray(feature.observations[cid], dtype=float)
            if len(m) < 2 or not np.all(np.isfinite(m[:2])):
                return None
            b_cam = np.array([m[0], m[1], 1.0], dtype=float)
            n = np.linalg.norm(b_cam)
            if not np.isfinite(n) or n <= 0.0:
                return None
            b_cam /= n
            R_w_c0 = to_rotation(cam_state.orientation)
            b_world = R_w_c0.T @ b_cam
            n = np.linalg.norm(b_world)
            if not np.isfinite(n) or n <= 0.0:
                return None
            return b_world / n

        def motion_stats(ids, prefix):
            if len(ids) < 2:
                return
            first_id = ids[0]
            last_id = ids[-1]

            first_state = self.state_server.cam_states[first_id]
            last_state = self.state_server.cam_states[last_id]

            baseline = np.linalg.norm(last_state.position - first_state.position)
            stats[f"{prefix}_baseline"] = float(baseline)

            b0 = bearing_world(first_id)
            b1 = bearing_world(last_id)
            if b0 is None or b1 is None:
                return

            dot = float(np.clip(b0 @ b1, -1.0, 1.0))
            stats[f"{prefix}_parallax_deg"] = float(np.degrees(np.arccos(dot)))

        motion_stats(all_ids, "all")
        motion_stats(used_ids, "used")

        return stats

    def _log_feature_geometry(self, feature, cam_state_ids, H_xj, r_j, accepted, context):
        if os.getenv("VIO_DIAGNOSTICS") != "1":
            return

        diag_dir = os.getenv("VIO_DIAGNOSTICS_DIR")
        if not diag_dir:
            return

        try:
            import csv
            from pathlib import Path

            row = {
                "timestamp": self._diag_timestamp,
                "context": context,
                "gate_accepted": int(bool(accepted)),
                "H_rows": int(H_xj.shape[0]) if hasattr(H_xj, "shape") else 0,
                "H_cols": int(H_xj.shape[1]) if hasattr(H_xj, "shape") and len(H_xj.shape) > 1 else 0,
                "r_len": int(len(r_j)) if hasattr(r_j, "__len__") else 0,
                "r_norm": float(np.linalg.norm(r_j)) if hasattr(r_j, "__len__") and len(r_j) > 0 and np.all(np.isfinite(r_j)) else 0.0,
            }
            row.update(self._feature_geometry_stats(feature, cam_state_ids))

            path = Path(diag_dir) / "feature_geometry.csv"
            path.parent.mkdir(parents=True, exist_ok=True)

            fields = list(row.keys())
            write_header = not path.exists()
            with path.open("a", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                if write_header:
                    writer.writeheader()
                writer.writerow(row)

        except Exception as exc:
            print(f"[diagnostics] failed to write feature_geometry.csv: {exc}")


    def remove_lost_features(self):
        # Удаляет признаки, потерявшие трекинг.
        # Определяет размер итоговой матрицы якобиана и вектора невязок.
        jacobian_row_size = 0
        invalid_feature_ids = []
        processed_feature_ids = []

        for feature in self.map_server.values():
            # Пропускает признаки, которые ещё отслеживаются.
            if self.state_server.imu_state.id in feature.observations:
                continue
            if len(feature.observations) < 3:
                invalid_feature_ids.append(feature.id)
                continue

            if not feature.is_initialized:
                if not feature.check_motion(self.state_server.cam_states):
                    invalid_feature_ids.append(feature.id)
                    continue

                ret = feature.initialize_position(self.state_server.cam_states)
                if ret is False:
                    invalid_feature_ids.append(feature.id)
                    continue

            jacobian_row_size += (4 * len(feature.observations) - 3)
            processed_feature_ids.append(feature.id)

        for feature_id in invalid_feature_ids:
            del self.map_server[feature_id]

        if len(processed_feature_ids) == 0:
            return

        H_x = np.zeros((jacobian_row_size, 
            21+6*len(self.state_server.cam_states)))
        r = np.zeros(jacobian_row_size)
        stack_count = 0

        for feature_id in processed_feature_ids:
            feature = self.map_server[feature_id]

            cam_state_ids = []
            for cam_id, measurement in feature.observations.items():
                cam_state_ids.append(cam_id)

            H_xj, r_j = self.feature_jacobian(feature.id, cam_state_ids)

            accepted = self.gating_test(H_xj, r_j)
            self._log_feature_geometry(feature, cam_state_ids, H_xj, r_j, accepted,
                                       "remove_lost_features")
            if accepted:
                H_x[stack_count:stack_count+H_xj.shape[0], :H_xj.shape[1]] = H_xj
                r[stack_count:stack_count+len(r_j)] = r_j
                stack_count += H_xj.shape[0]

            if stack_count > 1500:
                break

        H_x = H_x[:stack_count]
        r = r[:stack_count]

        self.measurement_update(H_x, r)

        for feature_id in processed_feature_ids:
            del self.map_server[feature_id]

    def find_redundant_cam_states(self):
        cam_state_pairs = list(self.state_server.cam_states.items())

        key_cam_state_idx = len(cam_state_pairs) - 4
        cam_state_idx = key_cam_state_idx + 1
        first_cam_state_idx = 0

        key_position = cam_state_pairs[key_cam_state_idx][1].position
        key_rotation = to_rotation(
            cam_state_pairs[key_cam_state_idx][1].orientation)

        rm_cam_state_ids = []

        for i in range(2):
            position = cam_state_pairs[cam_state_idx][1].position
            rotation = to_rotation(
                cam_state_pairs[cam_state_idx][1].orientation)
            
            distance = np.linalg.norm(position - key_position)
            cos_angle = np.clip(to_quaternion(rotation @ key_rotation.T)[-1], -1.0, 1.0)
            angle = 2 * np.arccos(cos_angle)

            if angle < 0.2618 and distance < 0.4 and self.tracking_rate > 0.5:
                rm_cam_state_ids.append(cam_state_pairs[cam_state_idx][0])
                cam_state_idx += 1
            else:
                rm_cam_state_ids.append(cam_state_pairs[first_cam_state_idx][0])
                first_cam_state_idx += 1
                cam_state_idx += 1

        rm_cam_state_ids = sorted(rm_cam_state_ids)
        return rm_cam_state_ids


    def prune_cam_state_buffer(self):
        if len(self.state_server.cam_states) < self.config.max_cam_state_size:
            return

        rm_cam_state_ids = self.find_redundant_cam_states()

        jacobian_row_size = 0
        for feature in self.map_server.values():

            involved_cam_state_ids = []
            for cam_id in rm_cam_state_ids:
                if cam_id in feature.observations:
                    involved_cam_state_ids.append(cam_id)

            if len(involved_cam_state_ids) == 0:
                continue
            if len(involved_cam_state_ids) == 1:
                del feature.observations[involved_cam_state_ids[0]]
                continue

            if not feature.is_initialized:
                if not feature.check_motion(self.state_server.cam_states):
                    for cam_id in involved_cam_state_ids:
                        del feature.observations[cam_id]
                    continue

                ret = feature.initialize_position(self.state_server.cam_states)
                if ret is False:
                    for cam_id in involved_cam_state_ids:
                        del feature.observations[cam_id]
                    continue

            jacobian_row_size += 4*len(involved_cam_state_ids) - 3

        H_x = np.zeros((jacobian_row_size, 21+6*len(self.state_server.cam_states)))
        r = np.zeros(jacobian_row_size)

        stack_count = 0
        for feature in self.map_server.values():
            involved_cam_state_ids = []
            for cam_id in rm_cam_state_ids:
                if cam_id in feature.observations:
                    involved_cam_state_ids.append(cam_id)

            if len(involved_cam_state_ids) == 0:
                continue

            H_xj, r_j = self.feature_jacobian(feature.id, involved_cam_state_ids)

            accepted = self.gating_test(H_xj, r_j)
            self._log_feature_geometry(feature, involved_cam_state_ids, H_xj, r_j, accepted,
                                       "prune_cam_state_buffer")
            if accepted:
                H_x[stack_count:stack_count+H_xj.shape[0], :H_xj.shape[1]] = H_xj
                r[stack_count:stack_count+len(r_j)] = r_j
                stack_count += H_xj.shape[0]

            for cam_id in involved_cam_state_ids:
                del feature.observations[cam_id]

        H_x = H_x[:stack_count]
        r = r[:stack_count]

        self.measurement_update(H_x, r)

        for cam_id in rm_cam_state_ids:
            idx = list(self.state_server.cam_states.keys()).index(cam_id)
            cam_state_start = 21 + 6*idx
            cam_state_end = cam_state_start + 6

            state_cov = self.state_server.state_cov.copy()
            if cam_state_end < state_cov.shape[0]:
                size = state_cov.shape[0]
                state_cov[cam_state_start:-6, :] = state_cov[cam_state_end:, :]
                state_cov[:, cam_state_start:-6] = state_cov[:, cam_state_end:]
            self.state_server.state_cov = state_cov[:-6, :-6]

            del self.state_server.cam_states[cam_id]

    def reset_state_cov(self):
        """
        Сбрасывает ковариацию состояния.
        """
        state_cov = np.zeros((21, 21))
        state_cov[ 3: 6,  3: 6] = self.config.gyro_bias_cov * np.identity(3)
        state_cov[ 6: 9,  6: 9] = self.config.velocity_cov * np.identity(3)
        state_cov[ 9:12,  9:12] = self.config.acc_bias_cov * np.identity(3)
        state_cov[15:18, 15:18] = self.config.extrinsic_rotation_cov * np.identity(3)
        state_cov[18:21, 18:21] = self.config.extrinsic_translation_cov * np.identity(3)
        self.state_server.state_cov = state_cov

    def reset(self):
        """
        Сбрасывает VIO в начальное состояние.
        """
        imu_state = IMUState()
        imu_state.id = self.state_server.imu_state.id
        imu_state.R_imu_cam0 = self.state_server.imu_state.R_imu_cam0
        imu_state.t_cam0_imu = self.state_server.imu_state.t_cam0_imu
        self.state_server.imu_state = imu_state

        self.state_server.cam_states.clear()

        self.reset_state_cov()

        self.map_server.clear()

        self.imu_msg_buffer.clear()

        self.is_gravity_set = False
        self.is_first_img = True

    def online_reset(self):
        """
        Онлайн-сброс системы при слишком большой неопределённости.
        """

        if self.config.position_std_threshold <= 0:
            return

        position_x_std = np.sqrt(self.state_server.state_cov[12, 12])
        position_y_std = np.sqrt(self.state_server.state_cov[13, 13])
        position_z_std = np.sqrt(self.state_server.state_cov[14, 14])

        if max(position_x_std, position_y_std, position_z_std 
            ) < self.config.position_std_threshold:
            return

        print('Start online reset...')

        self.state_server.cam_states.clear()

        self.map_server.clear()

        self.reset_state_cov()

    def publish(self, time):
        imu_state = self.state_server.imu_state
        print('+++publish:')
        print('   timestamp:', imu_state.timestamp)
        print('   orientation:', imu_state.orientation)
        print('   position:', imu_state.position)
        print('   velocity:', imu_state.velocity)
        print()
        
        T_i_w = Isometry3d(
            to_rotation(imu_state.orientation).T,
            imu_state.position)
        T_b_w = IMUState.T_imu_body * T_i_w * IMUState.T_imu_body.inverse()
        body_velocity = IMUState.T_imu_body.R @ imu_state.velocity

        R_w_c = imu_state.R_imu_cam0 @ T_i_w.R.T
        t_c_w = imu_state.position + T_i_w.R @ imu_state.t_cam0_imu
        T_c_w = Isometry3d(R_w_c.T, t_c_w)

        self._write_state(imu_state)
        
        return namedtuple('vio_result', ['timestamp', 'pose', 'velocity', 'cam0_pose'])(
            time, T_b_w, body_velocity, T_c_w)
