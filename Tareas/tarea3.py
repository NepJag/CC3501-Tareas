import pyglet
from OpenGL import GL
import numpy as np
import sys
import os
from Box2D import b2PolygonShape, b2World
# No es necesario este bloque de código si se ejecuta desde la carpeta raíz del repositorio
# v
if sys.path[0] != "":
    sys.path.insert(0, "")
sys.path.append('../../')
# ^
sys.path.append(os.path.dirname(os.path.dirname((os.path.abspath(__file__)))))
# No es necesario este bloque de código si se ejecuta desde la carpeta raíz del repositorio

import auxiliares.utils.shapes as shapes
from auxiliares.utils.camera import OrbitCamera
from auxiliares.utils.scene_graph import SceneGraph
from auxiliares.utils.drawables import Model, Texture, DirectionalLight, Material, SpotLight
from auxiliares.utils.helpers import init_pipeline, mesh_from_file, get_path

WIDTH, HEIGHT = 800, 800

class Controller(pyglet.window.Window):
    def __init__(self, title, *args, **kargs):
        super().__init__(*args, **kargs)
        self.set_minimum_size(240, 240) # Evita error cuando se redimensiona a 0
        self.set_caption(title)
        self.key_handler = pyglet.window.key.KeyStateHandler()
        self.push_handlers(self.key_handler)
        self.program_state = { "total_time": 0.0, "camera": None }
        self.init()

    def init(self):
        GL.glClearColor(0, 0, 0, 1.0)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_CULL_FACE)
        GL.glCullFace(GL.GL_BACK)
        GL.glFrontFace(GL.GL_CCW)

    def is_key_pressed(self, key):
        return self.key_handler[key]
    
if __name__ == "__main__":

    # La ventana
    controller = Controller("Tarea 3", width=WIDTH, height=HEIGHT, resizable=True)

    controller.program_state["camera"] = OrbitCamera(5, "perspective")
    controller.program_state["camera"].phi = -np.pi/4
    controller.program_state["camera"].theta = np.pi / 3
    camera = controller.program_state["camera"]
    
    color_mesh_lit_pipeline = init_pipeline(
        get_path("auxiliares/shaders/color_mesh_lit.vert"),
        get_path("auxiliares/shaders/color_mesh_lit.frag"))
    
    textured_mesh_lit_pipeline = init_pipeline(
        get_path("auxiliares/shaders/textured_mesh_lit.vert"),
        get_path("auxiliares/shaders/textured_mesh_lit.frag"))
    
    color_mesh_lit_pipeline2 = init_pipeline(
        get_path("auxiliares/shaders/color_mesh_lit.vert"),
        get_path("auxiliares/shaders/color_mesh_lit.frag"))
    
    textured_mesh_lit_pipeline2 = init_pipeline(
        get_path("auxiliares/shaders/textured_mesh_lit.vert"),
        get_path("auxiliares/shaders/textured_mesh_lit.frag"))
    
    # Meshes
    quad = Model(shapes.Square["position"], shapes.Square["uv"], shapes.Square["normal"], index_data=shapes.Square["indices"])
    platform = mesh_from_file(get_path("Tareas/tech_pedestal.obj"))[0]["mesh"]
    RB6 = mesh_from_file(get_path("Tareas/RB6.stl"))[0]["mesh"]
    RB6_FW = mesh_from_file(get_path("Tareas/RB6_front_wheel.stl"))[0]["mesh"]
    RB6_RW = mesh_from_file(get_path("Tareas/RB6_rear_wheel.stl"))[0]["mesh"]

    graph = SceneGraph(controller)
    graph.add_node("sun",
                    pipeline=[color_mesh_lit_pipeline, textured_mesh_lit_pipeline],
                    light=DirectionalLight(),
                    rotation=[-np.pi/4, 0, 0],
                   )
    graph.add_node("spotlight1",
                   pipeline=[color_mesh_lit_pipeline, textured_mesh_lit_pipeline],
                   position=[-2, 1, -2],
                   rotation=[-3*np.pi/4, np.pi/4, 0],
                   light=SpotLight(
                          diffuse = [1, 0, 0],
                          specular = [1, 0, 0],
                          ambient = [0.2, 0, 0],
                          cutOff = 0.91, # siempre mayor a outerCutOff
                          outerCutOff = 0.82
                   )
                )
    graph.add_node("spotlight2",
            pipeline=[color_mesh_lit_pipeline, textured_mesh_lit_pipeline],
            position=[2, 1, -2],
            rotation=[-3*np.pi/4, -np.pi/4, 0],
            light=SpotLight(
                diffuse = [0, 1, 0],
                specular = [0, 1, 0],
                ambient = [0, 0.2, 0],
                cutOff = 0.91, # siempre mayor a outerCutOff
                outerCutOff = 0.82
                )
            )
    graph.add_node("spotlight3",
            pipeline=[color_mesh_lit_pipeline, textured_mesh_lit_pipeline],
            position=[2, 1, 2],
            rotation=[-3*np.pi/4, 5*np.pi/4, 0],
            light=SpotLight(
                diffuse = [0, 0, 1],
                specular = [0, 0, 1],
                ambient = [0, 0, 0.2],
                cutOff = 0.91, # siempre mayor a outerCutOff
                outerCutOff = 0.82
                )
            )
    graph.add_node("spotlight4",
            pipeline=[color_mesh_lit_pipeline, textured_mesh_lit_pipeline],
            position=[-2, 1, 2],
            rotation=[-3*np.pi/4, 3*np.pi/4, 0],
            light=SpotLight(
                diffuse = [1, 1, 0],
                specular = [1, 1, 0],
                ambient = [0.2, 0.2, 0],
                cutOff = 0.91, # siempre mayor a outerCutOff
                outerCutOff = 0.82
                )
            )
    graph.add_node("floor",
                   mesh = quad,
                   pipeline = textured_mesh_lit_pipeline,
                   rotation = [-np.pi/2, 0, 0],
                   texture=Texture(r"CC3501-Tareas\Tareas\black-stones-tiled-floor.jpg",
                                    sWrapMode=GL.GL_REPEAT, 
                                    tWrapMode=GL.GL_REPEAT),
                   scale = [50, 30, 1],
                   material = Material(shininess=64))
    graph.add_node("wall1",
                   mesh = quad,
                   pipeline = textured_mesh_lit_pipeline,
                   position= [0, 2.5, -15],
                   rotation = [0, 0, 0],
                   texture=Texture(r"CC3501-Tareas\Tareas\wall1.jpg"),
                   scale = [50, 5, 1],
                   material = Material(shininess=64),
                   )
    graph.add_node("wall2",
                   mesh = quad,
                   pipeline = textured_mesh_lit_pipeline,
                   position= [0, 2.5, 15],
                   rotation = [0, np.pi, 0],
                   texture=Texture(r"CC3501-Tareas\Tareas\wall1.jpg"),
                   scale = [50, 5, 1],
                   material = Material(shininess=64),
                   )
    graph.add_node("platform",
                   mesh= platform,
                   pipeline = textured_mesh_lit_pipeline,
                   position=[0, 0.05, 0],
                   rotation=[0, 0, 0],
                   scale=[2.5, 0.3, 2.5],
                   material = Material(diffuse=[0.75, 0.75, 0.75]),
                   )
    
    # Number of cars (max 10)
    n = 4
    wheel_mat = Material(specular=[0.3, 0.3, 0.3], diffuse=[0.1,0.1,0.1], shininess=64)
    origin = [0, 0.3, 0]

    # Ten teams colors
    colors = np.array([[166, 5, 26], # Ferrari
                       [0, 19, 68], # Red Bull
                       [255, 128, 0], # McLaren
                       [13, 14, 14], # Mercedes
                       [249, 242, 242], # Haas
                       [32, 57, 76], # Alpha Tauri
                       [152, 30, 50], # Alfa Romeo
                       [0, 160, 222], # Williams
                       [33, 115, 184], # Alpine
                       [3, 122, 104] # Aston Martin
                       ])
    colors = colors / 255

    shininess = np.array([32, 64, 16, 128, 256, 64, 64, 16, 8, 16])

    graph.add_node("Cars")
    for i in range(n):
        graph.add_node(f"RB6_{i}",
                       mesh= RB6,
                       attach_to="Cars",
                       pipeline = color_mesh_lit_pipeline,
                       position=[origin[0] + 6*i, origin[1], origin[2]],
                       rotation=[-np.pi/2, 0, 0],
                       scale=[1.5, 1.5, 1.5],
                       material = Material(diffuse=colors[i], shininess=shininess[i]),
                       )
        graph.add_node(f"RB6_{i}_FW1",
                    attach_to=f"RB6_{i}",
                    mesh= RB6_FW,
                    pipeline = color_mesh_lit_pipeline,
                    position=[0.28, -0.61, 0.03],
                    rotation=[0, 0, 0],
                    scale=[0.3, 0.3, 0.3],
                    material = wheel_mat,
                    )
        graph.add_node(f"RB6_{i}_FW2",
                    attach_to=f"RB6_{i}",
                    mesh= RB6_FW,
                    pipeline = color_mesh_lit_pipeline,
                    position=[-0.28, -0.61, 0.03],
                    rotation=[0, np.pi, 0],
                    scale=[0.3, 0.3, 0.3],
                    material = wheel_mat,
                    )
        graph.add_node(f"RB6_{i}_RW1",
                    attach_to=f"RB6_{i}",
                    mesh= RB6_RW,
                    pipeline = color_mesh_lit_pipeline,
                    position=[0.3, 0.65, 0.03],
                    scale=[0.35, 0.3, 0.3],
                    material = wheel_mat,
                    )
        graph.add_node(f"RB6_{i}_RW2",
                    attach_to=f"RB6_{i}",
                    mesh= RB6_RW,
                    pipeline = color_mesh_lit_pipeline,
                    position=[-0.3, 0.65, 0.03],
                    rotation=[0, np.pi, 0],
                    scale=[0.35, 0.3, 0.3],
                    material = wheel_mat,
                    )

    cars = graph["Cars"]

    # Animation duration 
    duration = 1

    # Target position
    target_pos = None

    # Tween progress  
    tween = 0

    # Selected car
    selected_car = None
    play_graph = SceneGraph(controller)
    play_graph.add_node("sun",
                    pipeline=[color_mesh_lit_pipeline2, textured_mesh_lit_pipeline2],
                    light=DirectionalLight(),
                    rotation=[-np.pi/4, 0, 0],
                   )
    play_graph.add_node("floor",
                   mesh = quad,
                   pipeline = textured_mesh_lit_pipeline2,
                   rotation = [-np.pi/2, 0, 0],
                   texture=None,
                   scale = [50, 30, 1],
                   material = Material(shininess=64))

    @controller.event
    def on_key_press(symbol, modifiers):
        global target_pos, tween, selected_car
        
        # Switch car
        if symbol == pyglet.window.key.SPACE:
            
            # Animation
            target_pos = (cars["position"][0] - 6) % (-6 * n)
            if target_pos == -0:
                target_pos = 0
        
        # Select car
        elif symbol == pyglet.window.key.ENTER:

            i = int(np.absolute(cars["position"][0] // 6))
            selected_car = f"RB6_{i}"
            play_graph.add_node(selected_car, 
                                attach_to="root", 
                                mesh= RB6,
                                pipeline = color_mesh_lit_pipeline2,
                                position=[origin[0], origin[1], origin[2]],
                                rotation=[-np.pi/2, 0, 0],
                                scale=[1.5, 1.5, 1.5],
                                material = Material(diffuse=colors[i], shininess=shininess[i]),
                                )
            play_graph.add_node(f"RB6_{i}_FW1", 
                                attach_to="root",
                                mesh= RB6_FW,
                                pipeline = color_mesh_lit_pipeline2,
                                position=[0.4, 0.3, 0.9],
                                rotation=[0, 0, 0],
                                scale=[0.4, 0.4, 0.4],
                                material = wheel_mat,
                                )
            play_graph.add_node(f"RB6_{i}_FW2",
                                mesh= RB6_FW,
                                pipeline = color_mesh_lit_pipeline2,
                                position=[-0.4, 0.3, 0.9],
                                rotation=[0, np.pi, 0],
                                scale=[0.4, 0.4, 0.4],
                                material = wheel_mat,
                                )
            play_graph.add_node(f"RB6_{i}_RW1",
                                mesh= RB6_RW,
                                pipeline = color_mesh_lit_pipeline2,
                                position=[0.4, 0.3, -1],
                                scale=[0.45, 0.4, 0.4],
                                material = wheel_mat,
                                )
            play_graph.add_node(f"RB6_{i}_RW2",
                                mesh= RB6_RW,
                                pipeline = color_mesh_lit_pipeline2,
                                position=[-0.4, 0.3, -1],
                                rotation=[0, np.pi, 0],
                                scale=[0.45, 0.4, 0.4],
                                material = wheel_mat,
                                )
            camera.distance = 3
            camera.phi = np.pi
            camera.theta = np.pi/3
            
    def update(dt):
        
        global tween, target_pos
        
        # Animation
        if target_pos is not None:
            tween = min(tween + dt, duration) / duration
            t = tween
            cars["position"][0] = (1 - t) * cars["position"][0] + t * target_pos

            if tween == 1:
                target_pos = None
                tween = 0

        if selected_car is None:
            camera.phi += dt/2            
        
        camera.update()

    @controller.event
    def on_resize(width, height):
        camera.resize(width, height)

    @controller.event
    def on_draw():
        controller.clear()
        if selected_car is not None:
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT | GL.GL_STENCIL_BUFFER_BIT)
            controller.clear()
            play_graph.draw()
        else:
            graph.draw()

    pyglet.clock.schedule_interval(update, 1/60)
    pyglet.app.run()