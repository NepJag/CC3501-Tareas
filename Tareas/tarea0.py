import pyglet
from OpenGL import GL
import numpy as np

WIDTH = 800
HEIGHT = 800

class Controller(pyglet.window.Window):
    def __init__(self, title, *args, **kargs):
        super().__init__(*args, **kargs)
        self.set_minimum_size(240, 240)
        self.set_caption(title)

    def update(self, dt):
        pass

if __name__ == "__main__":

    controller = Controller("Tarea 0", width=WIDTH, height=HEIGHT, resizable=True)

    # Código del vertex shader
    # cada vértice tiene 3 atributos
    # posición (x, y)
    # color (r, g, b)
    # intensidad
    vertex_source_code = """
        #version 330

        in vec2 position;
        in vec3 color;
        in float intensity;

        out vec3 fragColor;
        out float fragIntensity;

        void main()
        {
            fragColor = color;
            fragIntensity = intensity;
            gl_Position = vec4(position, 0.0f, 1.0f);
        }
    """

    # Código del fragment shader
    # La salida es un vector de 4 componentes (r, g, b, a)
    # donde a es la transparencia (por ahora no nos importa, se deja en 1)
    # El color resultante de cada fragmento ("pixel") es el color del vértice multiplicado por su intensidad
    fragment_source_code = """
        #version 330

        in vec3 fragColor;
        in float fragIntensity;
        out vec4 outColor;

        void main()
        {
            outColor = fragIntensity * vec4(fragColor, 1.0f);
        }
    """

    vert_shader = pyglet.graphics.shader.Shader(vertex_source_code, "vertex")
    frag_shader = pyglet.graphics.shader.Shader(fragment_source_code, "fragment")

    pipeline = pyglet.graphics.shader.ShaderProgram(vert_shader, frag_shader)

    # Posición de los vértices de 3 triángulos formando una pirámide
    positions = np.array([
        -0.5, -0.5,
         0.5, -0.5, 
         0.0, -0.15,
         
         -0.5, -0.5,
         0.0,  0.5,
         0.0, -0.15,

         0.5, -0.5,
         0.0, -0.15,
         0.0, 0.5
    ], dtype=np.float32)

    # Colores de los vértices
    colors = np.array([
        0.3, 0, 0,
        0.3, 0, 0,
        1, 0, 0,

        0, 0.3, 0,
        0, 0.3, 0,
        0, 1, 0,

        0, 0, 0.3,
        0, 0, 1,
        0, 0, 0.3
    ], dtype=np.float32)

    # Intensidad de los vértices de los tríangulos
    intensities = np.array([
        1, 1, 1, 1, 1, 1, 1, 1, 1
    ], dtype=np.float32)

    indices = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8], dtype=np.uint32)

    # Creación de los vértices de los triángulos para uso en gpu
    gpu_triangle = pipeline.vertex_list_indexed(9, GL.GL_TRIANGLES, indices)
    gpu_triangle.position = positions
    gpu_triangle.color = colors
    gpu_triangle.intensity = intensities

    @controller.event
    def on_draw():
        # color de fondo al limpiar un frame
        GL.glClearColor(0.2, 0.2, 0.2, 1.0)
        # si hay algo dibujado se limpia del frame
        controller.clear()
        # se le dice al pipeline que se va a usar
        pipeline.use()
        # se le entrega al pipeline los vértices del triángulo
        gpu_triangle.draw(GL.GL_TRIANGLES)

    pyglet.clock.schedule_interval(controller.update, 1/60) # se ejecuta update del controller cada 1/60 segundos
    pyglet.app.run() # se ejecuta pyglet
