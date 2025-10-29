use macroquad::prelude::*;

struct Boid {
    pos: Vec2,
    vel: Vec2
}

impl Boid {
    fn new(x: f32, y: f32) -> Self {
        Self {
            pos: Vec2::new(x, y),
            vel: Vec2::ZERO
        }
    }
}

struct World {
    boids: Vec<Boid>
}

impl World {
    fn new() -> Self {
        Self { boids: vec![] }
    }

    fn handle_click(&mut self) {
        if is_mouse_button_pressed(MouseButton::Left) {
            let (x, y) = mouse_position();
            self.boids.push(Boid::new(x, y))
        }
    }

    fn draw(&self) {
        for boid in &self.boids {
            draw_circle(boid.pos.x, boid.pos.y, 10.0, BLUE);
        }
    }

    
}


#[macroquad::main("Swarm Sim")]
async fn main() {
    let mut world = World::new();

    loop {
        clear_background(BLACK);
        next_frame().await;
    }
}