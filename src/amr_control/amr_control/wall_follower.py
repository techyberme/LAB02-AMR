import math

class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    def __init__(self, dt: float):
        """Wall following class initializer.

        Args:
            dt: Sampling period [s].

        """
        self._dt: float = dt

    def compute_commands(self, z_us: list[float], z_v: float, z_w: float) -> tuple[float, float]:
        """Wall following exploration algorithm.

        Args:
            z_us: Distance from every ultrasonic sensor to the closest obstacle [m].
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
# TODO: 1.14. Complete the function body with your code (i.e., compute v and w).

        kp = 0.6      # coeficiente proporcional
        kd = 0.8       # coeficiente derivada

        d_ref = 0.45  # referencia distancia a pared
        v_nom = 0.3  # velocidad nominal
        w_max = 2     # velocidad angular máxima

        front_trigger = 0.55    # a 0.55m frena
        valid_max = 1.0         # máximo valor de los US

        wall_th_on  = 0.60  # si < esto, hay pared
        wall_th_off = 0.75  # si > esto, no hay pared

        # Memoria
        if not hasattr(self, "state"):
            self.state = 0

        if not hasattr(self, "lin_acc"):
            self.lin_acc = 0.0

        if not hasattr(self, "ang_acc"):
            self.ang_acc = 0.0

        if not hasattr(self, "turn_dir"):
            self.turn_dir = 0   # -1 derecha, +1 izquierda

        if not hasattr(self, "follow_side"):
            self.follow_side = 0  # 0=centrado, +1=izquierda, -1=derecha

        if not hasattr(self, "side_hold"):
            self.side_hold = 0.0

        if not hasattr(self, "has_left"):
            self.has_left = False

        if not hasattr(self, "has_right"):
            self.has_right = False

        # Sensores
        front_idx =  [3, 4]         # frontales
        right_idx = [6, 7, 8, 9]    # derecha
        left_idx  = [0, 1, 14, 15]  # izquierda

        # Función para limitar los valores recibidos poor los US
        def clamp_us(v):
            # Si no detecta (inf / <=0) -> lo tratamos como libre (1.0)
            if v is None:
                return valid_max
            if isinstance(v, float) and math.isinf(v):
                return valid_max
            if v <= 0.0:
                return valid_max
            # Limita al rango del sensor
            return min(v, valid_max)

        left_list  = [clamp_us(z_us[i]) for i in left_idx]   # valores en sensores izquierda
        right_list = [clamp_us(z_us[i]) for i in right_idx]  # valores en sensores derecha
        front_list = [clamp_us(z_us[i]) for i in front_idx]  # valores en sensores delante

        d_front_min = min(front_list, default=valid_max)     # valor delante más cercano
        d_front_max = max(front_list, default=valid_max)     # valor delante más lejano

        # Función para calcular media de sensores
        def mean(ds):
            return sum(ds) / len(ds) if ds else valid_max
        
        d_left_mean = mean(left_list)   # valores media izquierda
        d_right_mean = mean(right_list) # valores media derecha

        # Función robusta para calcular la media de los mínimos (daba problemas sin usar media)
        def robust_min(ds):
            ds = sorted(ds)
            return sum(ds[:2]) / 2.0  # promedio de los 2 más cercanos

        d_left_min  = robust_min(left_list)
        d_right_min = robust_min(right_list)

        # Detectar distancia hacia los lados
        d_left_free  = max(left_list,  default=valid_max)
        d_right_free = max(right_list, default=valid_max)

        # thresholds para comprobar si hay pared
        left_free_th = 0.65
        right_free_th = 0.65

        # Detectar si se puede considerar que no hay pared
        n_free_L = sum(1 for x in left_list  if x > left_free_th)
        n_free_R = sum(1 for x in right_list if x > right_free_th)

        # Si más de dos sensores detectan que no hay pared conlcuimos que podemos girar
        canL = n_free_L >= 2
        canR = n_free_R >= 2

        # Comprobamos si estamos pararlelos a las paredes
        parallel_right = bool(abs(right_list[1]-right_list[2])<0.02)
        parallel_left = bool(abs(left_list[0]-left_list[3])<0.02)

        # Hay paredes a los lados
        side_wall_th = 0.6

        # Si hay paredes a ambos lados y delante, condición para u-turn
        dead_end = (d_front_min < front_trigger) and (d_left_mean < side_wall_th) and (d_right_mean < side_wall_th)

        # Mirar hacia que lado hay pared y guardar en el estado de si hay
        if self.has_left:
            self.has_left = (d_left_min < wall_th_off)
        else:
            self.has_left = (d_left_min < wall_th_on)

        if self.has_right:
            self.has_right = (d_right_min < wall_th_off)
        else:
            self.has_right = (d_right_min < wall_th_on)

        # Camnbio de estado y debugging
        def enter_state(new_state):
            if self.state != new_state:
                print(f"[FSM] {self.state}->{new_state} | dF_min={d_front_min:.2f} dF_max={d_front_max:.2f} dL_min={d_left_min:.2f} dR_max={d_right_free:.2f} dL_max={d_left_free:.2f} dR_min={d_right_min:.2f}", flush=True)
                print(f"[front_list] {front_list}, [right_list] {right_list}, [left_list] {left_list}", flush=True)
            self.state = new_state
            self.lin_acc = 0.0
            self.ang_acc = 0.0
            if hasattr(self, "_e_prev"):
                delattr(self, "_e_prev")
            if hasattr(self, "_e_f"):
                delattr(self, "_e_f")

        # Máquina de estados 
        # 0: seguir paredes (PD)
        # 10: avance 0.5 m (fase 1)
        # 11: giro 90º (fase 2)
        # 20: U-turn 180º (con paredes laterales)

        v = 0.0
        w = 0.0

        if self.state == 0:
            v = v_nom # velocidad a velocidad nominal

            # Aguanta la pared que estaba siguiendo durante side_hold (tiempo)
            # Lo usamos para evitar que al salir del giro haga movimientos raros
            if self.side_hold > 0.0:
                self.side_hold -= self._dt
            else:
                # Auto selección de que pared debe seguir
                # Si puede sigue las dos y si no hay pared intenta seguir recto
                if self.has_left and self.has_right:
                    self.follow_side = 0
                elif self.has_left and (not self.has_right):
                    self.follow_side = +1
                elif self.has_right and (not self.has_left):
                    self.follow_side = -1
                else:
                    self.follow_side = 0  # sin paredes

            # Si no tiene paredes, inicializamos y mantenemos el error a 0
            if (not self.has_left) and (not self.has_right):
                e_raw = 0.0
            else:
                # Mantener distancia entre paredes
                if self.follow_side == 0:
                    e_raw = d_left_min - d_right_min
                # Mantener distancia de ref con la izquierda
                elif self.follow_side == +1:
                    e_raw = d_left_min - d_ref
                # Mantener distancia de ref con la derecha
                else:  # -1
                    e_raw = d_ref - d_right_min
        
            # filtre del error ponderado con error filtrado anterior y el error actual
            alpha_e = 0.15
            if not hasattr(self, "_e_f"):
                self._e_f = e_raw
            else:
                self._e_f = (1 - alpha_e) * self._e_f + alpha_e * e_raw
            e = self._e_f

            # derivada del error sobre el error filtrado
            if not hasattr(self, "_e_prev"):
                self._e_prev = e
            de = (e - self._e_prev) / self._dt
            de = max(-0.3, min(0.3, de))
            self._e_prev = e            

            # Controlador PD
            # Si no tiene pared izquierda ni derecha, sigue recto
            if (not self.has_left) and (not self.has_right):
                w = 0.0
            else:
                # PD
                w = kp*e + kd*de

            # clamp de la velocidad angular
            w = max(-w_max, min(w_max, w))

            # devuelve velocidad lineal y angular
            print(
                f"[DBG] st={self.state} side={self.follow_side} "
                f"hasL={int(self.has_left)} hasR={int(self.has_right)} "
                f"dLmin={d_left_min:.3f} dRmin={d_right_min:.3f} dFmin={d_front_min:.3f} "
                f"e_raw={e_raw:.4f} e_f={e:.4f} dt={self._dt:.3f} "
                f"z_w={z_w:.3f} "
                f"w_cmd={(kp*e + kd*de):.3f} w={w:.3f}",
                flush=True
                )

            # Detección de esquinas y decisión de giro
            if d_front_min < front_trigger:
                # Si hay dead-end hace u-turn
                if dead_end:
                    # Entra modo u-turn
                    enter_state(20)
                else:
                    if canL or canR:
                        if canL and canR:
                            # Para asegurar que gira cuando detecta un hueco, solo giramos al lado donde hay más distancia, y si no se decide derecha
                            if d_left_free > d_right_free:
                                self.turn_dir = +1
                            elif d_right_free > d_left_free:
                                self.turn_dir = -1
                            else:
                                self.turn_dir = -1
                        elif canL:
                            self.turn_dir = +1
                        else:
                            self.turn_dir = -1

                        # fija pared guía tras el giro
                        if self.turn_dir == -1:
                            self.follow_side = +1  # tras girar derecha, sigue izquierda
                        else:
                            self.follow_side = -1  # tras girar izquierda, sigue derecha
                        # Entra modo giro
                        enter_state(11)
                    else:
                        # Entra modo u-turn
                        enter_state(20)

        elif self.state == 11:
            v = 0.0 # velocidad a 0 para girar

            # angulo target 90º
            ang_target = math.pi / 2.0
            base_w = 1
            min_w = 0.3

            # cuando queda poco giro frena para no pasarse
            remaining = ang_target - self.ang_acc
            if remaining < 0.1:
                w_mag = max(min_w, base_w * (remaining / 0.3))
            else:
                w_mag = base_w

            w = self.turn_dir * w_mag
            w = max(-w_max, min(w_max, w))

            # Integrar para detectar 90º, solo sale del giro cuando detecta que está paralelo con la pared y hay hueco delante
            self.ang_acc += abs(w) * self._dt
            if (remaining <= 0) and (d_front_min > 0.98) and ((parallel_right and self.turn_dir == 1) or (parallel_left and self.turn_dir == -1)):
                if hasattr(self, "_e_prev"):
                    delattr(self, "_e_prev")
                # Mantener la pared que sigue durante 0.5
                self.side_hold = 0.5
                enter_state(0)

        elif self.state == 20:
            v = 0.0
            
            # ángulo target 180º
            ang_target = math.pi
            base_w = 1
            min_w = 0.3
            
            # Freno para no pasarse
            remaining = ang_target - self.ang_acc
            if remaining < 0.1:
                w_mag = max(min_w, base_w * (remaining / 0.3))
            else:
                w_mag = base_w

            w = self.turn_dir * w_mag
            w = max(-w_max, min(w_max, w))

            # Integrar hasta 180º, paralelo a las paredes, y con hueco delante
            self.ang_acc += abs(w) * self._dt
            if (remaining <= 0) and (d_front_min >= 0.98) and parallel_right and parallel_left:
                if hasattr(self, "_e_prev"):
                    delattr(self, "_e_prev")
                enter_state(0)

        w = max(-w_max, min(w_max, w))

        return v, w

