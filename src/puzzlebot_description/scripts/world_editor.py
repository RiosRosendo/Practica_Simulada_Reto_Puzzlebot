#!/usr/bin/env python3
"""
Puzzlebot World Editor
Sliders para ajustar el arena y los racks en tiempo real con preview 2D.
Guarda directamente en warehouse.world — luego relanza para ver los cambios.

Uso:
    python3 world_editor.py
"""

import tkinter as tk
from tkinter import ttk, messagebox
import math
import os

# Ruta al world file (relativa al home)
WORLD_PATH = os.path.expanduser(
    "~/puzzlebot_nav_ws/src/Puzzlebot-Challenge-/src/"
    "puzzlebot_description/worlds/warehouse.world"
)

# Dimensiones del rack a escala=1.0 (metros)
RACK_W = 0.30    # ancho (X)
RACK_L = 0.875   # largo (Y)

RACK_COLORS = ['#f0a030', '#30a0f0', '#a030f0', '#30f060', '#f03060']


class RackControls:
    """Sliders para un rack."""
    def __init__(self, parent, index, x=0.0, y=0.0, yaw=0.0, scale=1.0):
        self.index = index
        self.x     = tk.DoubleVar(value=x)
        self.y     = tk.DoubleVar(value=y)
        self.yaw   = tk.DoubleVar(value=yaw)    # grados
        self.scale = tk.DoubleVar(value=scale)

        color = RACK_COLORS[index % len(RACK_COLORS)]
        frame = ttk.LabelFrame(parent, text=f" Rack {index+1} ",
                               padding=4)
        frame.pack(fill='x', pady=3)

        _row(frame, "X (m)",       self.x,     -3.0,  3.0, 0.01)
        _row(frame, "Y (m)",       self.y,     -3.0,  3.0, 0.01)
        _row(frame, "Rotación °",  self.yaw,    0.0, 360.0, 1.0)
        _row(frame, "Tamaño",      self.scale,  0.3,  3.0,  0.05)


def _row(parent, label, var, lo, hi, res):
    """Una fila: etiqueta + slider + valor."""
    f = ttk.Frame(parent)
    f.pack(fill='x', pady=1)
    ttk.Label(f, text=label, width=12, anchor='w').pack(side='left')
    ttk.Scale(f, from_=lo, to=hi, variable=var,
              orient='horizontal', length=160).pack(side='left', padx=4)
    lbl = ttk.Label(f, text=f"{var.get():.2f}", width=6, anchor='e')
    lbl.pack(side='left')
    var.trace_add('write', lambda *_: lbl.configure(text=f"{var.get():.2f}"))


class WorldEditor(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title("Puzzlebot World Editor")
        self.resizable(True, True)

        # Variables del arena
        self.arena_x = tk.DoubleVar(value=3.0)
        self.arena_y = tk.DoubleVar(value=2.5)

        self.racks: list[RackControls] = []

        self._build_ui()

        # Redibuja cada vez que algo cambia
        for var in (self.arena_x, self.arena_y):
            var.trace_add('write', lambda *_: self._draw())

        self._draw()

    # ── UI ──────────────────────────────────────────────────────────────

    def _build_ui(self):
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        # ── Panel izquierdo: controles ───────────────────────────────
        left = ttk.Frame(self, padding=8)
        left.grid(row=0, column=0, sticky='nsew')

        # Arena
        arena_frame = ttk.LabelFrame(left, text=" Arena ", padding=4)
        arena_frame.pack(fill='x', pady=(0, 6))
        _row(arena_frame, "Ancho X (m)", self.arena_x, 1.0, 6.0, 0.05)
        _row(arena_frame, "Largo Y (m)", self.arena_y, 1.0, 6.0, 0.05)

        # Contenedor scrollable para racks
        self._rack_container = ttk.Frame(left)
        self._rack_container.pack(fill='x')

        # Racks por defecto
        self._add_rack(0.5,  0.5,   0.0, 1.0)
        self._add_rack(0.5, -0.5,   0.0, 1.0)
        self._add_rack(-0.5, 0.3,  90.0, 1.0)

        # Botones
        btn = ttk.Frame(left)
        btn.pack(fill='x', pady=8)
        ttk.Button(btn, text="+ Agregar Rack",
                   command=lambda: self._add_rack()).pack(fill='x', pady=2)
        ttk.Button(btn, text="− Quitar último Rack",
                   command=self._remove_rack).pack(fill='x', pady=2)
        ttk.Separator(btn).pack(fill='x', pady=4)
        ttk.Button(btn, text="💾  Guardar warehouse.world",
                   command=self._save).pack(fill='x', pady=2)

        # ── Canvas: preview 2D ───────────────────────────────────────
        self.canvas = tk.Canvas(self, bg='#1a1a2e', width=520, height=520)
        self.canvas.grid(row=0, column=1, padx=8, pady=8, sticky='nsew')

        # Nota inferior
        note = ttk.Label(self,
            text="Guarda → relanza con: "
                 "ros2 launch puzzlebot_navigation autonomous_nav.launch.py "
                 "world_name:=warehouse spawn_obstacles:=false",
            foreground='gray', wraplength=700)
        note.grid(row=1, column=0, columnspan=2, pady=(0, 6))

    def _add_rack(self, x=0.0, y=0.0, yaw=0.0, scale=1.0):
        rc = RackControls(self._rack_container,
                          len(self.racks), x, y, yaw, scale)
        for var in (rc.x, rc.y, rc.yaw, rc.scale):
            var.trace_add('write', lambda *_: self._draw())
        self.racks.append(rc)
        self._draw()

    def _remove_rack(self):
        if not self.racks:
            return
        # Destruir el último LabelFrame
        self._rack_container.winfo_children()[-1].destroy()
        self.racks.pop()
        self._draw()

    # ── Preview 2D ─────────────────────────────────────────────────────

    def _draw(self, *_):
        if not hasattr(self, 'canvas'):
            return
        c = self.canvas
        c.delete('all')

        W = c.winfo_width()  or 520
        H = c.winfo_height() or 520
        pad = 48

        aw = self.arena_x.get()
        al = self.arena_y.get()
        ppm = min((W - 2*pad) / aw, (H - 2*pad) / al)   # pixels per metre

        def px(x): return W / 2 + x * ppm
        def py(y): return H / 2 - y * ppm

        # Fondo arena
        c.create_rectangle(px(-aw/2), py(-al/2), px(aw/2), py(al/2),
                            outline='#cccccc', width=2, fill='#16213e')

        # Grid cada 0.5 m
        for v in [i * 0.5 for i in range(-20, 21)]:
            if -aw/2 <= v <= aw/2:
                c.create_line(px(v), py(-al/2), px(v), py(al/2),
                              fill='#2a2a4a', dash=(3, 5))
            if -al/2 <= v <= al/2:
                c.create_line(px(-aw/2), py(v), px(aw/2), py(v),
                              fill='#2a2a4a', dash=(3, 5))

        # Robot (origen)
        r = 8
        c.create_oval(px(0)-r, py(0)-r, px(0)+r, py(0)+r,
                      fill='#44ff44', outline='white', width=1)
        c.create_text(px(0), py(0) - r - 8, text="robot",
                      fill='#aaffaa', font=('Courier', 8))

        # Dimensiones
        c.create_text(px(aw/2) - 4, py(0), text=f"{aw/2:.2f}m",
                      fill='#ff6666', anchor='e', font=('Courier', 8))
        c.create_text(px(0), py(al/2) + 4, text=f"{al/2:.2f}m",
                      fill='#66ff66', anchor='n', font=('Courier', 8))

        # Racks
        for rc in self.racks:
            rx, ry  = rc.x.get(), rc.y.get()
            yaw_rad = math.radians(rc.yaw.get())
            s       = rc.scale.get()
            rw      = RACK_W * s
            rl      = RACK_L * s
            col     = RACK_COLORS[rc.index % len(RACK_COLORS)]

            corners = [(-rw/2, -rl/2), (rw/2, -rl/2),
                       (rw/2,  rl/2), (-rw/2,  rl/2)]
            pts = []
            for lx, ly in corners:
                gx = rx + lx * math.cos(yaw_rad) - ly * math.sin(yaw_rad)
                gy = ry + lx * math.sin(yaw_rad) + ly * math.cos(yaw_rad)
                pts += [px(gx), py(gy)]

            c.create_polygon(pts, fill=col, outline=col, width=2, stipple='gray25')
            # Flecha de dirección (eje largo)
            fx = rx + (rl/2) * (-math.sin(yaw_rad))
            fy = ry + (rl/2) * ( math.cos(yaw_rad))
            c.create_line(px(rx), py(ry), px(fx), py(fy),
                          fill=col, width=2, arrow='last')
            c.create_text(px(rx), py(ry), text=str(rc.index + 1),
                          fill='white', font=('Courier', 10, 'bold'))

    # ── Guardar world ───────────────────────────────────────────────────

    def _save(self):
        aw = self.arena_x.get()
        al = self.arena_y.get()

        racks_sdf = ''
        for rc in self.racks:
            rx   = rc.x.get()
            ry   = rc.y.get()
            yaw  = math.radians(rc.yaw.get())
            s    = rc.scale.get()
            coll_h  = 0.331 * s
            box_x   = 0.30  * s
            box_y   = 0.875 * s
            box_z   = 0.6625 * s
            vis_ox  = 0.15   * s
            vis_oy  = 0.3375 * s
            mesh_sc = 0.025  * s
            racks_sdf += f"""
    <model name="rack_{rc.index + 1}">
      <static>true</static>
      <pose>{rx:.3f} {ry:.3f} 0 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <pose>0 0 {coll_h:.4f} 0 0 0</pose>
          <geometry><box><size>{box_x:.4f} {box_y:.4f} {box_z:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <pose>{vis_ox:.4f} {vis_oy:.4f} 0 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>model://full_rack/meshes/full_rack.stl</uri>
              <scale>{mesh_sc:.5f} {mesh_sc:.5f} {mesh_sc:.5f}</scale>
            </mesh>
          </geometry>
          <material>
            <ambient>0.6 0.5 0.3 1</ambient>
            <diffuse>0.7 0.6 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

        world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">

    <physics name="1ms" type="ode">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Arena {aw:.2f} x {al:.2f} m — generado por world_editor.py -->

    <model name="wall_north">
      <static>true</static><pose>0 {al/2:.3f} 0.3 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{aw:.3f} 0.1 0.6</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{aw:.3f} 0.1 0.6</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
      </link>
    </model>

    <model name="wall_south">
      <static>true</static><pose>0 {-al/2:.3f} 0.3 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{aw:.3f} 0.1 0.6</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{aw:.3f} 0.1 0.6</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
      </link>
    </model>

    <model name="wall_east">
      <static>true</static><pose>{aw/2:.3f} 0 0.3 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.1 {al:.3f} 0.6</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.1 {al:.3f} 0.6</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
      </link>
    </model>

    <model name="wall_west">
      <static>true</static><pose>{-aw/2:.3f} 0 0.3 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.1 {al:.3f} 0.6</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.1 {al:.3f} 0.6</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
      </link>
    </model>
{racks_sdf}
  </world>
</sdf>
"""
        with open(WORLD_PATH, 'w') as f:
            f.write(world)

        messagebox.showinfo(
            "Guardado",
            f"warehouse.world actualizado.\n\n"
            f"Relanza con:\n"
            f"ros2 launch puzzlebot_navigation autonomous_nav.launch.py "
            f"world_name:=warehouse spawn_obstacles:=false"
        )


if __name__ == '__main__':
    WorldEditor().mainloop()
