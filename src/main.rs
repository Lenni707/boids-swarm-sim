use macroquad::prelude::*;

const SCREEN_HEIGHT: i32 = 800;
const SCREEN_WIDTH: i32 = 800;

const VISUAL_RANGE: f32 = 100.0;
const COHERENCE: f32 = 0.1;
const AVOIDFACTOR: f32 = 0.1;
const AVOIDDISTANCE: f32 = 50.0;
const ALLIGNMENTFACTOR: f32 = 0.1;

#[derive(PartialEq)]
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
    
    fn alignment(&mut self) -> Vec<Vec2> { // boids versuchen den speed der umliegenden boids (im sichtfeld) zu matchen
        let mut changed_vels = vec![];
        for self_boid in &self.boids {
            let mut boids_vel: Vec2 = Vec2::ZERO;
            let mut neighbors: f32 = 0.0;
            for other_boid in &self.boids {
                let distance = self_boid.pos.distance(other_boid.pos);
                if distance <= VISUAL_RANGE {
                    boids_vel += other_boid.vel;
                    neighbors += 1.0
                }
            }
            if neighbors > 0.0 {
                let new_boid_speed = (boids_vel / neighbors) - self_boid.vel * ALLIGNMENTFACTOR; // also der durchschnittspeed minus den self boid durch den alligment faktor heißt er Versucht sich an den durchschnitt anzupassen
                changed_vels.push(new_boid_speed)
            }
            else {
                changed_vels.push(Vec2::ZERO)
            }
        }
        changed_vels
    }

    fn separation(&self) -> Vec<Vec2> { // boids wollen sich nicht zu nahe kommen
        let mut changed_vels = vec![];
        for self_boid in &self.boids {
            let mut move_away = Vec2::ZERO;
            for other_boid in &self.boids {
                if self_boid == other_boid {
                    continue;
                }
                let distance = self_boid.pos.distance(other_boid.pos); // feature von macroqaud, basicly satz des pythagoras mit den vectoren
                if distance < AVOIDDISTANCE {
                    // je näher desto mehr wollen die weg
                    move_away += (self_boid.pos - other_boid.pos) * AVOIDFACTOR;
                }
            }
            changed_vels.push(move_away);
        }
        changed_vels
    }

    fn cohesion(&mut self) -> Vec<Vec2> { // boids steuern richtung durschschnittliche koordinaten der umliegenden boids (im sichtfeld)
        let mut changed_vels = vec![];
        for self_boid in &self.boids {
            let mut boids_pos: Vec2 = Vec2::ZERO;
            let mut neighbors: f32 = 0.0;
            for other_boid in &self.boids {
                let distance = self_boid.pos.distance(other_boid.pos);
                if distance <= VISUAL_RANGE {
                    boids_pos += other_boid.pos;
                    neighbors += 1.0
                }
            }
            if neighbors > 0.0 {
                let boids_avr_pos = boids_pos / neighbors;

                let new_boid_speed = (boids_avr_pos - self_boid.pos) * COHERENCE; // boid versucht zur durchschnittlichen position zu steuern

                changed_vels.push(new_boid_speed)
            }
            else {
                changed_vels.push(Vec2::ZERO)
            }
        }
        changed_vels
    }

    fn check_edge(&mut self) -> Vec<Vec2> {
        let mut move_away = Vec2::ZERO;
        let mut changed_vels = vec![];
        for boid in &self.boids {
            if boid.pos.x > SCREEN_WIDTH as f32 - 20.0 {
                let dist = boid.pos.x - (SCREEN_WIDTH as f32 - 20.0);
                move_away += Vec2::new(-dist * 0.2, 0.0); // push left
            } else if boid.pos.x < 20.0 {
                let dist = 20.0 - boid.pos.x;
                move_away += Vec2::new(dist * 0.2, 0.0); // push right
            }

            if boid.pos.y > SCREEN_HEIGHT as f32 - 20.0 {
                let dist = boid.pos.y - (SCREEN_HEIGHT as f32 - 20.0);
                move_away += Vec2::new(0.0, -dist * 0.2); // push up
            } else if boid.pos.y < 20.0 {
                let dist = 20.0 - boid.pos.y;
                move_away += Vec2::new(0.0, dist * 0.2); // push down
            }
            changed_vels.push(move_away);
        }
        changed_vels
    }

    fn update_velocities(&mut self) -> Vec<Vec2> { // added alles zusammen
        let alignment = self.alignment();
        let separation = self.separation();
        let cohesion = self.cohesion();
        let edge_avoid = self.check_edge();

        let mut final_vels = vec![Vec2::ZERO; self.boids.len()]; // vektor direkt mit richtiger länge

        for i in 0..self.boids.len() {
            final_vels[i] = alignment[i] + separation[i] + cohesion[i] + edge_avoid[i];
        }

        final_vels
    }

    fn update(&mut self) {
        let updated_vels = self.update_velocities();
        for i in 0..self.boids.len() {
            self.boids[i].pos += updated_vels[i]
        }
    }
}

fn window_conf() -> Conf {
    Conf {
        window_title: "Swarm Sim".to_string(),
        window_width: SCREEN_WIDTH,
        window_height: SCREEN_HEIGHT,
        fullscreen: false,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut world = World::new();

    loop {
        clear_background(BLACK);

        world.handle_click();
        world.update();
        world.draw();

        next_frame().await;
    }
}