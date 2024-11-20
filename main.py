import json
import math
import customtkinter as ctk
import mujoco as mj
import mujoco.glfw as glfw
import numpy as np


#Clase para las esferas, incluye masa, radio y color
class Ball:

    def __init__(self, radius, mass, color):
        
        self.radius = radius
        self.mass = mass
        self.color = color
        self.position = 0
        self.velocity = 0
    
    def update_position (self, time_step, acceleration):

        """Funcion encargada de actualizar la posicion,
        y velocidad de la esfera, esta incluye los parametros
        time_step(marca el paso del tiempo) y aceleracion"""

        self.velocity += acceleration * time_step
        self.position += self.velocity * time_step


#Clase para rampas, tiene en cuenta el angulo de la rampa, longitud y friccion
class Ramp:

    def __init__(self, angle, length, friction):
        
        self.angle = angle
        self.length = length
        self.friction = friction
    
    def caclulate_acceleration(self, gravity=9.81):

        """Encargada de calcular la aceleracion en la rampa
        tiene en cuenta la gravedad definida localmente"""

        return gravity * math.sin(math.radians(self.angle)) -(self.friction * gravity * math.cos(math.radians(self.angle)))


class Simulator:

    def __init__(self, path):

        self.button_left = False
        self.button_middle = False
        self.button_right = False
        self.lastx = 0
        self.lasty = 0

        #Checkea la inicializacion del programa
        if not glfw.init():
            raise RuntimeError("Error initializing GLFW")

        self.window = glfw.create_window(1200, 900, "MuJoCo Viewer", None, None)

        #Checkea la ventana
        if not self.window:
            glfw.terminate()
            raise RuntimeError("Error creating window")
        
        glfw.make_context_current(self.window)

        #Habilita el V-Sync
        glfw.swap_interval(1) 


        #Carga modelos y datos de mujoco
        self.model = mj.MjModel.from_xml_path(path)
        self.data = mj.MjData(self.model)


        #Configura la camara, opciones y renderizado
        self.cam = mj.MjvCamera()
        self.opt = mj.MjvOption()
        self.scene = mj.MjvScene(self.model, maxgeom=10000)
        self.context = mj.MjrContext(self.model, mj.mjtFontScale.mjFONTSCALE_150.value)
        mj.mjv_defaultCamera(self.cam)
        mj.mjv_defaultOption(self.opt)


        #Configura callbacks del raton
        glfw.set_key_callback(self.window, self.keyboard)
        glfw.set_cursor_pos_callback(self.window, self.mouse_move)
        glfw.set_mouse_button_callback(self.window, self.mouse_button)
        glfw.set_scroll_callback(self.window, self.scroll)


        #identifica el objeto a mover
        self.object_name = "esfera"
        self.object_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, self.object_name)


    def keyboard(self, window, key, scancode, act, mods):

        """Maneja los eventos en el teclado y restablece 
        el simulador con la tecla BACKSPACE"""

        if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
            mj.mj_resetData(self.model, self.data)
            mj.mj_forward(self.model, self.data)


    def mouse_move(self, window, xpos, ypos):

        """Maneja los eventos de movimiento del
        raton y ajusta la camara segun su movimiento"""

        dx= xpos - self.lastx
        dy = ypos - self.lasty

        self.lastx = xpos
        self.lasty = ypos

        if not (self.button_left or self.button_middle or self.button_right):
            return
        

        width, height = glfw.get_window_size(window)
        mod_shift = (glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS or
                      glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)
        
        if self.button_right:
            action = mj.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mj.mjtMouse.mjMOUSE_MOVE_V
        elif self.button_left:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mj.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mj.mjtMouse.mjMOUSE_ZOOM

        mj.mjv_moveCamera(self.model, action, dx / height, dy / height, self.scene, self.cam)


    def scroll(self, window, xoffset, yoffset):

        """maneja eventos de desplazamiento del raton 
        y ajusta el zoom de la camara"""

        action = mj.mjtMouse.mjMOUSE_ZOOM
        mj.mjv_moveCamera(self.mode, action, 0.0, -0.05 * yoffset, self.scene,self.cam)

    def mouse_button(self, window, button, act, mods):

        """Maneja los eventos de los botones del raton"""

        self.button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
        self.button_middle= (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
        self.button_right= (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    def update_object_position(self):

        """Actualiza la posicion del objeto seleccionado
        mediante el uso de coordenadas"""

        if self.object_id is not None and self.button_left:

            scale_factor = 0.001
            new_position = np.array([
                (self.mouse_x - 600) * scale_factor,
                (450 - self.mouse_y) * scale_factor,
                0.2 #Se mantiene z constante
            ])

            self.model.geom_pos[self.object_id] = new_position

    def run(self):

        """Bucle principal de la simulacion"""

        while not (glfw.window_should_close(self.window)):
            mj.mjstep(self.model, self.data)
            mj.mjforward(self.model, self.data)

            if self.button_left:
                self.update_object_position()
            
            mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam, mj.mjtCAtBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(mj.MjrRect(0, 0, 1200, 900), self.scene, self.context)

            glfw.swap_buffers(self.window)
            glfw.poll_events()

        glfw.terminate()

    def button_callback():
        """
        Callback para iniciar la simulacion desde la interfaz grafica.
        """
        sim = Simulator("esfera_rampa.xml")
        sim.run()


# Configuración de la interfaz grafica
app = customtkinter.CTk()
app.title("Simulador Esferas")
app.geometry("400x150")

# Botones
button1 = customtkinter.CTkButton(app, text="Iniciar Simulación", command=button_callback)
button1.grid(row=0, column=0, padx=20, pady=20)

app.mainloop()
