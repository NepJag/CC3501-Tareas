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

import grafica.transformations as tr
import auxiliares.utils.shapes as shapes
from auxiliares.utils.camera import OrbitCamera, FreeCamera
from auxiliares.utils.scene_graph import SceneGraph
from auxiliares.utils.drawables import Model, Texture, DirectionalLight, Material, SpotLight
from auxiliares.utils.helpers import init_pipeline, mesh_from_file, get_path

WIDTH, HEIGHT = 600, 600

class Controller(pyglet.window.Window):
    def __init__(self, title, *args, **kargs):
        super().__init__(*args, **kargs)
        self.set_minimum_size(240, 240) # Evita error cuando se redimensiona a 0
        self.set_caption(title)
        self.keys_state = {}
        self.program_state = { 
            "total_time": 0.0, 
            "camera": None ,
            "bodies": {},
            "world": None,
            # parámetros para el integrador
            "vel_iters": 6,
            "pos_iters": 2 }
        self.init()

    def init(self):
        GL.glClearColor(0, 0, 0, 1.0)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_CULL_FACE)
        GL.glCullFace(GL.GL_BACK)
        GL.glFrontFace(GL.GL_CCW)

    def is_key_pressed(self, key):
        return self.keys_state.get(key, False)

    def on_key_press(self, symbol, modifiers):
        controller.keys_state[symbol] = True
        super().on_key_press(symbol, modifiers)

    def on_key_release(self, symbol, modifiers):
        controller.keys_state[symbol] = False
    
if __name__ == "__main__":

    # La ventana
    controller = Controller("Tarea 3", width=WIDTH, height=HEIGHT, resizable=True)

    controller.program_state["camera"] = OrbitCamera(5, "perspective")
    controller.program_state["camera"].phi = -np.pi/4
    controller.program_state["camera"].theta = np.pi/3
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
                   texture=Texture(get_path("Tareas/black-stones-tiled-floor.jpg"),
                                    sWrapMode=GL.GL_REPEAT, 
                                    tWrapMode=GL.GL_REPEAT),
                   scale = [50, 30, 1],
                   material = Material(shininess=64))
    graph.add_node("wall1",
                   mesh = quad,
                   pipeline = textured_mesh_lit_pipeline,
                   position= [0, 2.5, -15],
                   rotation = [0, 0, 0],
                   texture=Texture(get_path("Tareas/wall1.jpg")),
                   scale = [50, 5, 1],
                   material = Material(shininess=64),
                   )
    graph.add_node("wall2",
                   mesh = quad,
                   pipeline = textured_mesh_lit_pipeline,
                   position= [0, 2.5, 15],
                   rotation = [0, np.pi, 0],
                   texture=Texture(get_path("Tareas/wall1.jpg")),
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
    
    world = b2World(gravity=(0, 0))
    controller.program_state["world"] = world

    chassis = world.CreateDynamicBody(position=(0, 0), angle=0, linearDamping=0.75, angularDamping=3.5)
    chassis.CreatePolygonFixture(box=(0.25, 0.5), density=1, friction=0.3)
    controller.program_state["bodies"]["chassis"] = chassis

    # 4 wheels
    wheels = []

    wheel_pos = [(1.0, 0.5),
                 (-1.0, 0.5),
                 (1.0, -0.5),
                 (-1.0, -0.5)
                ]
    
    for i in range(4):
        wheel = world.CreateDynamicBody(position=wheel_pos[i], angle=0, linearDamping=0.5, angularDamping=0.5)
        wheel.CreatePolygonFixture(box=(0.25, 0.25), density=1, friction=0.3)
        controller.program_state["bodies"][f"wheel{i}"] = wheel
        wheels.append(wheel)
    
    # [0.4, 0.3, 0.9], [-0.4, 0.3, 0.9], [0.4, 0.3, -1], [-0.4, 0.3, -1]

    wheel_anchors = [(1.0, 0.5), 
                     (-1.0, 0.5),
                     (1.0, -0.5),
                     (-1.0, -0.5)]

    world.CreateWheelJoint(bodyA=chassis, bodyB=wheels[0], 
                           anchor=wheel_anchors[0], collideConnected=True)
    world.CreateWheelJoint(bodyA=chassis, bodyB=wheels[1], 
                           anchor=wheel_anchors[1], collideConnected=True)
    world.CreateWheelJoint(bodyA=chassis, bodyB=wheels[2], 
                           anchor=wheel_anchors[2], collideConnected=True)
    world.CreateWheelJoint(bodyA=chassis, bodyB=wheels[3], 
                           anchor=wheel_anchors[3], collideConnected=True)                                                      

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
            play_graph.add_node(selected_car)
            play_graph.add_node("visual_RB6", 
                                attach_to=selected_car, 
                                mesh= RB6,
                                pipeline = color_mesh_lit_pipeline2,
                                position=[0.0, 0.3, 0.0],
                                rotation=[-np.pi/2, 0, 0],
                                scale=[1.5, 1.5, 1.5],
                                material = Material(diffuse=colors[i], shininess=shininess[i]),
                                )
            play_graph.add_node(f"RB6_{i}_FW1", 
                                attach_to="root",
                                mesh= RB6_FW,
                                pipeline = color_mesh_lit_pipeline2,
                                position=[0.0, 0.3, 0.0],
                                rotation=[0, 0, 0],
                                scale=[0.4, 0.4, 0.4],
                                material = wheel_mat,
                                )
            play_graph.add_node(f"RB6_{i}_FW2")
            play_graph.add_node("visual_RB6_FW2",
                                mesh= RB6_FW,
                                attach_to=f"RB6_{i}_FW2",
                                pipeline = color_mesh_lit_pipeline2,
                                position=[0.0, 0.3, 0.0],
                                rotation=[0, np.pi, 0],
                                scale=[0.4, 0.4, 0.4],
                                material = wheel_mat,
                                )
            play_graph.add_node(f"RB6_{i}_RW1",
                                mesh= RB6_RW,
                                pipeline = color_mesh_lit_pipeline2,
                                position=[0.0, 0.3, 0.0],
                                scale=[0.45, 0.4, 0.4],
                                material = wheel_mat,
                                )
            play_graph.add_node(f"RB6_{i}_RW2")
            play_graph.add_node("visual_RB6_RW2",
                                attach_to=f"RB6_{i}_RW2",
                                mesh= RB6_RW,
                                pipeline = color_mesh_lit_pipeline2,
                                position=[0.0, 0.3, 0.0],
                                rotation=[0, np.pi, 0],
                                scale=[0.45, 0.4, 0.4],
                                material = wheel_mat,
                                )
            
            controller.program_state["camera"] = FreeCamera([0, 0, 0], "perspective")
            controller.program_state["camera"].yaw = np.pi/3
    
    def update_world(dt):
        controller.program_state["total_time"] += dt
        controller.program_state["world"].Step(dt, controller.program_state["vel_iters"], controller.program_state["pos_iters"])

        play_graph[selected_car]["position"][0] = chassis.position[0]
        play_graph[selected_car]["position"][2] = chassis.position[1]
        play_graph[selected_car]["rotation"][1] = -chassis.angle

        for i in range(2):
            play_graph[selected_car+f"_FW{i+1}"]["position"][0] = wheels[i].position[0]
            play_graph[selected_car+f"_FW{i+1}"]["position"][2] = wheels[i].position[1]
            play_graph[selected_car+f"_FW{i+1}"]["rotation"][1] = -wheels[i].angle

            play_graph[selected_car+f"_RW{i+1}"]["position"][0] = wheels[i+2].position[0]
            play_graph[selected_car+f"_RW{i+1}"]["position"][2] = wheels[i+2].position[1]
            play_graph[selected_car+f"_RW{i+1}"]["rotation"][1] = -wheels[i+2].angle

    def update(dt):

        camera = controller.program_state["camera"]
        controllable = controller.program_state["bodies"]["chassis"]

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
        
        elif selected_car is not None:
            update_world(dt)
            if controller.is_key_pressed(pyglet.window.key.W):
                forward = play_graph.get_forward(selected_car)
                controllable.ApplyForceToCenter((forward[0]*10, forward[2]*10), True)

            if controller.is_key_pressed(pyglet.window.key.S):
                forward = play_graph.get_forward(selected_car)
                controllable.ApplyForceToCenter((-forward[0]*10, -forward[2]*10), True)

            if controller.is_key_pressed(pyglet.window.key.D):
                controllable.angularVelocity = 1

            if controller.is_key_pressed(pyglet.window.key.A):
                controllable.angularVelocity = -1
            
            if controller.is_key_pressed(pyglet.window.key._1):
                camera.type = "perspective"
                
            if controller.is_key_pressed(pyglet.window.key._2):
                camera.type = "orthographic"

            camera.position[0] = controllable.position[0] + 4 * np.sin(controllable.angle)
            camera.position[1] = 2
            camera.position[2] = controllable.position[1] - 4 * np.cos(controllable.angle)
            camera.yaw = controllable.angle + np.pi / 2

            print(camera.position)

        camera.update()

    @controller.event
    def on_resize(width, height):
        camera.resize(width, height)

    @controller.event
    def on_draw():
        controller.clear()
        if selected_car is not None:
            controller.clear()
            play_graph.draw()
        else:
            graph.draw()

    pyglet.clock.schedule_interval(update, 1/60)
    pyglet.app.run()