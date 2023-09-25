import pyglet
from OpenGL import GL
import numpy as np
import trimesh as tm
import networkx as nx
import os
import sys
from pathlib import Path

sys.path.append(os.path.dirname(os.path.dirname((os.path.abspath(__file__)))))
import grafica.transformations as tr
import auxiliares.utils.shapes as shapes

WIDTH, HEIGHT = 800, 800

class Controller(pyglet.window.Window):
    def __init__(self, title, *args, **kargs):
        super().__init__(*args, **kargs)
        self.set_minimum_size(240, 240)
        self.set_caption(title)
        self.key_handler = pyglet.window.key.KeyStateHandler()
        self.push_handlers(self.key_handler)
        self.init()
    
    def init(self):
        GL.glClearColor(0, 0, 0, 0)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_CULL_FACE)
        GL.glCullFace(GL.GL_BACK)
        GL.glFrontFace(GL.GL_CCW)

    def is_key_pressed(self, key):
        return self.key_handler[key]
    
class Model():
    def __init__(self, position_data, color_data, index_data=None):
        self.position_data = position_data
        self.color_data = color_data

        self.index_data = index_data
        if index_data is not None:
            self.index_data = np.array(index_data, dtype=np.uint32)

        self.gpu_data = None

        self.position = np.array([0, 0, 0], dtype=np.float32)
        self.rotation = np.array([0, 0, 0], dtype=np.float32)
        self.scale = np.array([1, 1, 1], dtype=np.float32)

    def init_gpu_data(self, pipeline):
        if self.index_data is not None:
            self.gpu_data = pipeline.vertex_list_indexed(len(self.position_data) // 3, GL.GL_TRIANGLES, self.index_data)
        else:
            self.gpu_data = pipeline.vertex_list(len(self.position_data) // 3, GL.GL_TRIANGLES)
        
        self.gpu_data.position[:] = self.position_data
        self.gpu_data.color[:] = np.array(self.color_data).flatten()

    def draw(self, mode = GL.GL_TRIANGLES):
        self.gpu_data.draw(mode)

    def get_transform(self):
        translation_matrix = tr.translate(self.position[0], self.position[1], self.position[2])
        rotation_matrix = tr.rotationX(self.rotation[0]) @ tr.rotationY(self.rotation[1]) @ tr.rotationZ(self.rotation[2])
        scale_matrix = tr.scale(self.scale[0], self.scale[1], self.scale[2])
        transformation = translation_matrix @ rotation_matrix @ scale_matrix
        return np.reshape(transformation, (16, 1), order="F")

class Mesh(Model):
    def __init__(self, asset_path, base_color=None):
        mesh_data = tm.load(asset_path)
        mesh_scale = tr.uniformScale(2.0 / mesh_data.scale)
        mesh_translate = tr.translate(*-mesh_data.centroid)
        mesh_data.apply_transform(mesh_scale @ mesh_translate)
        vertex_data = tm.rendering.mesh_to_vertexlist(mesh_data)
        indices = vertex_data[3]
        positions = vertex_data[4][1]

        count = len(positions) // 3
        colors = np.full((count * 3, 1), 1.0)
        if base_color is None:
            colors = vertex_data[5][1]
        else:
            for i in range(count):
                if i % 3 == 0:
                    colors[i*3] = base_color[0]
                    colors[i*3 + 1] = base_color[1]
                    colors[i*3 + 1] = base_color[2]
                elif i % 3 == 1:
                    colors[i*3] = base_color[0] + 0.1
                    colors[i*3 + 1] = base_color[1] + 0.1
                    colors[i*3 + 1] = base_color[2] + 0.1
                else:
                    colors[i*3] = base_color[0] + 0.3
                    colors[i*3 + 1] = base_color[1] + 0.3
                    colors[i*3 + 1] = base_color[2] + 0.3

        super().__init__(positions, colors, indices)

class Camera():
    def __init__(self, camera_type = "perspective"):
        self.position = np.array([1, 0, 0], dtype=np.float32)
        self.focus = np.array([0, 0, 0], dtype=np.float32)
        self.type = camera_type
    
    def update(self):
        pass

    def get_view(self):
        lookAt_matrix = tr.lookAt(self.position, self.focus, np.array([0, 1, 0]))
        return np.reshape(lookAt_matrix, (16, 1), order="F")
    
    def get_projection(self, width, height):
        if self.type == "perspective":
            perspective_matrix = tr.perspective(90, width/height, 0.01, 100)
        elif self.type == "orthographic":
            depth = self.position - self.focus
            depth = np.linalg.norm(depth)
            perspective_matrix = tr.ortho(-(width/height) * depth, (width/height) * depth, -1 * depth, 1 * depth, 0.01, 100)
        return np.reshape(perspective_matrix, (16, 1), order="F")

class OrbitCamera(Camera):
    def __init__(self, distance, camera_type = "perspective"):
        super().__init__(camera_type)
        self.distance = distance
        self.phi = 0
        self.theta = np.pi/2
        self.update()

    def update(self):
        if self.theta > np.pi:
            self.theta = np.pi
        elif self.theta < 0:
            self.theta = 0.0001
        
        self.position[0] = self.distance * np.sin(self.theta) * np.sin(self.phi)
        self.position[1] = self.distance * np.cos(self.theta)
        self.position[2] = self.distance * np.sin(self.theta) * np.cos(self.phi)

if __name__ == "__main__":

    # La ventana
    controller = Controller("Tarea 1", width=WIDTH, height=HEIGHT, resizable=True)

    with open(Path(os.path.dirname(__file__)) / "../auxiliares/shaders/transform.vert") as f:
        vertex_source_code = f.read()

    with open(Path(os.path.dirname(__file__)) / "../auxiliares/shaders/color.frag") as f:
        fragment_source_code = f.read()

    vert_shader = pyglet.graphics.shader.Shader(vertex_source_code, "vertex")
    frag_shader = pyglet.graphics.shader.Shader(fragment_source_code, "fragment")

    pipeline = pyglet.graphics.shader.ShaderProgram(vert_shader, frag_shader)
    
    camera = OrbitCamera(3, "perspective")
    camera.phi = np.pi / 4
    camera.theta = np.pi / 4
    
    axes = Model(shapes.Axes["position"], shapes.Axes["color"])
    axes.init_gpu_data(pipeline)

    RB6_mesh = Mesh(Path(os.path.dirname(__file__)) / "RB6.stl")
    RB6_mesh.init_gpu_data(pipeline)
    RB6_mesh.rotation = np.array([-np.pi/2, 0, 0])
    RB6_mesh.scale = np.array([1.5, 1.5, 1.5])

    # Front wheels    
    RB6_FW_mesh1 = Mesh(Path(os.path.dirname(__file__)) / "RB6_front_wheel.stl")
    RB6_FW_mesh1.init_gpu_data(pipeline)
    RB6_FW_mesh1.position = np.array([0.45, 0, 0.93])
    RB6_FW_mesh1.scale = np.array([0.4, 0.4, 0.4])

    RB6_FW_mesh2 = Mesh(Path(os.path.dirname(__file__)) / "RB6_front_wheel.stl")
    RB6_FW_mesh2.init_gpu_data(pipeline)
    RB6_FW_mesh2.rotation = np.array([0, np.pi, 0])
    RB6_FW_mesh2.position = np.array([-0.45, 0, 0.93])
    RB6_FW_mesh2.scale = np.array([0.4, 0.4, 0.4])

    # Rear wheels
    RB6_RW_mesh1 = Mesh(Path(os.path.dirname(__file__)) / "RB6_rear_wheel.stl")
    RB6_RW_mesh1.init_gpu_data(pipeline)
    RB6_RW_mesh1.position = np.array([0.45, 0.03, -0.93])
    RB6_RW_mesh1.scale = np.array([0.45, 0.45, 0.45])

    RB6_RW_mesh2 = Mesh(Path(os.path.dirname(__file__)) / "RB6_rear_wheel.stl")
    RB6_RW_mesh2.init_gpu_data(pipeline)
    RB6_RW_mesh2.rotation = np.array([0, np.pi, 0])
    RB6_RW_mesh2.position = np.array([-0.45, 0.03, -0.93])
    RB6_RW_mesh2.scale = np.array([0.45, 0.45, 0.45])

    # Garage
    garage_mesh = Mesh(Path(os.path.dirname(__file__)) / "cube.off", [0.2, 0.3, 0.5])
    garage_mesh.init_gpu_data(pipeline)
    garage_mesh.scale = np.array([5, 5, 5])
    garage_mesh.position = np.array([0, 2.63, 0])

    # Toolbox
    toolbox_mesh = Mesh(Path(os.path.dirname(__file__)) / "toolbox.obj")
    toolbox_mesh.init_gpu_data(pipeline)
    toolbox_mesh.rotation = np.array([0, np.pi/4, 0])
    toolbox_mesh.scale = np.array([0.4, 0.4, 0.4])
    toolbox_mesh.position = np.array([-1.5, -0.11, 0])

    def update(dt):

        # Uncomment for manual camera control
        # if controller.is_key_pressed(pyglet.window.key.A):
        #     camera.phi -= dt
        # if controller.is_key_pressed(pyglet.window.key.D):
        #     camera.phi += dt
        # if controller.is_key_pressed(pyglet.window.key.W):
        #     camera.theta -= dt
        # if controller.is_key_pressed(pyglet.window.key.S):
        #     camera.theta += dt
        # if controller.is_key_pressed(pyglet.window.key.Q):
        #     camera.distance += dt
        # if controller.is_key_pressed(pyglet.window.key.E):
        #     camera.distance -= dt
        # if controller.is_key_pressed(pyglet.window.key._1):
        #     camera.type = "perspective"
        # if controller.is_key_pressed(pyglet.window.key._2):
        #     camera.type = "orthographic"

        # Comment for manual camera control    
        camera.phi += dt/2
        
        camera.update()

    @controller.event
    def on_draw():
        controller.clear()
        pipeline.use()
        
        pipeline["u_view"] = camera.get_view()
        pipeline["u_projection"] = camera.get_projection(controller.width, controller.height)

        axes.draw(GL.GL_LINES)
        
        # Garage
        GL.glCullFace(GL.GL_FRONT)
        pipeline["u_model"] = garage_mesh.get_transform()
        garage_mesh.draw()
        GL.glCullFace(GL.GL_BACK)

        # Toolbox
        pipeline["u_model"] = toolbox_mesh.get_transform()
        toolbox_mesh.draw()

        # Front wheels
        pipeline["u_model"] = RB6_FW_mesh1.get_transform()
        RB6_FW_mesh1.draw()
        pipeline["u_model"] = RB6_FW_mesh2.get_transform()
        RB6_FW_mesh2.draw()

        # Rear wheels
        pipeline["u_model"] = RB6_RW_mesh1.get_transform()
        RB6_RW_mesh1.draw()
        pipeline["u_model"] = RB6_RW_mesh2.get_transform()
        RB6_RW_mesh2.draw()

        # Body
        pipeline["u_model"] = RB6_mesh.get_transform()
        RB6_mesh.draw()

    pyglet.clock.schedule_interval(update, 1/60)
    pyglet.app.run()