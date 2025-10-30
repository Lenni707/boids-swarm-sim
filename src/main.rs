use macroquad::prelude::*;
use macroquad::rand::gen_range;

// todo: Werter anpassen, jeweilige kraft limitieren damit eines nicht zu stark wird
// https://vanhunteradams.com/Pico/Animal_Movement/Boids-algorithm.html

const SCREEN_WIDTH: i32 = 1400;
const SCREEN_HEIGHT: i32 = 800;

const VISUAL_RANGE: f32 = 100.0;          
const COHERENCE: f32 = 0.005;            
const AVOIDFACTOR: f32 = 0.05;          
const AVOIDDISTANCE: f32 = 20.0;         
const ALIGNMENTFACTOR: f32 = 0.05;      

const TURNFACTOR: f32 = 0.4;
const EDGE_DISTANCE: f32 = 50.0;  

#[derive(PartialEq)]
struct Boid {
    pos: Vec2,
    vel: Vec2
}

impl Boid {
    fn new(x: f32, y: f32) -> Self {
        let vel = Vec2::new(
            rand::gen_range(-5.0, 5.0), // random start speed
            rand::gen_range(-5.0, 5.0),
        );
        
        Self {
            pos: Vec2::new(x, y),
            vel: vel
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
            draw_circle(boid.pos.x, boid.pos.y, 2.0, WHITE);
        }
    }
    
    fn alignment(&mut self) -> Vec<Vec2> { // boids versuchen den speed der umliegenden boids (im sichtfeld) zu matchen
        let mut changed_vels = vec![];
        for self_boid in &self.boids {
            let mut boids_vel: Vec2 = Vec2::ZERO;
            let mut neighbors: f32 = 0.0;
            for other_boid in &self.boids {
                let distance = self_boid.pos.distance(other_boid.pos);
                if distance <= VISUAL_RANGE && distance >= AVOIDDISTANCE { // hoffe das zweite hilft: && distance >= AVOIDDISTANCE 
                    boids_vel += other_boid.vel;
                    neighbors += 1.0
                }
            }
            if neighbors > 0.0 {
                let avg_speed = boids_vel / neighbors;
                let new_boid_speed = (avg_speed.normalize() - self_boid.vel.normalize())  * ALIGNMENTFACTOR; // also der durchschnittspeed minus den self boid durch den alligment faktor heißt er Versucht sich an den durchschnitt anzupassen
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
                if distance < AVOIDDISTANCE && distance > 0.0 {
                    let diff = self_boid.pos - other_boid.pos;
                    move_away += diff.normalize() / distance;  // stronger when close
                    
                }
            }
            move_away *= AVOIDFACTOR;
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
                if distance <= VISUAL_RANGE && distance >= AVOIDDISTANCE {
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

    fn check_edge(&self) -> Vec<Vec2> { // should work 
        let mut changed_vels = vec![];
        for boid in &self.boids {
            let mut move_away = Vec2::ZERO;
            if boid.pos.x > SCREEN_WIDTH as f32 - EDGE_DISTANCE {
                move_away.x -= TURNFACTOR
            } else if boid.pos.x < EDGE_DISTANCE {
                move_away.x += TURNFACTOR
            }

            if boid.pos.y > SCREEN_HEIGHT as f32 - EDGE_DISTANCE {
                move_away.y -= TURNFACTOR
            } else if boid.pos.y < EDGE_DISTANCE {
                move_away.y += TURNFACTOR
            }
            changed_vels.push(move_away);
        }
        changed_vels
    }

    fn update_velocities(&mut self) -> Vec<Vec2> { // added alles zusammen
        if self.boids.is_empty() { // damit kein panic wenn keine boids da
            return vec![];
        }

        let alignment = self.alignment();
        let separation = self.separation();
        let cohesion = self.cohesion();
        let edge_avoid = self.check_edge();

        let mut final_vels = vec![Vec2::ZERO; self.boids.len()]; // vektor direkt mit richtiger länge

        for i in 0..self.boids.len() {
            final_vels[i] =
                alignment[i].clamp_length_max(1.0) * 0.5 +
                separation[i].clamp_length_max(1.0) * 1.5 +
                cohesion[i].clamp_length_max(1.0) * 0.2 +
                edge_avoid[i].clamp_length_max(1.0) * 1.0;
        }

        final_vels
    }

    fn update(&mut self) {
        if self.boids.is_empty() {
            return;
        }

        let updated_vels = self.update_velocities();

        for i in 0..self.boids.len() {
            // neue velocity

            let old_vel = self.boids[i].vel;

            self.boids[i].vel += updated_vels[i];

            self.boids[i].vel += Vec2::new(gen_range(-0.1, 0.1), gen_range(-0.1, 0.1)); // bisschen randomizen

            // max speed
            let speed = self.boids[i].vel.length();
            let max_speed = 15.0;
            let min_speed = 5.0;
            if speed > max_speed {
                self.boids[i].vel = self.boids[i].vel.normalize() * max_speed;
            }
            if speed < min_speed {
                self.boids[i].vel = self.boids[i].vel.normalize() * min_speed;
            }

            let max_turn = 1.0; // how much they can turn per frame
            let vel_change = self.boids[i].vel - old_vel;
            if vel_change.length() > max_turn {
                self.boids[i].vel = old_vel + vel_change.normalize() * max_turn;
            }
            // update pos
            let vel = self.boids[i].vel;
            self.boids[i].pos += vel; 
        }
    }

    fn spawn_boids(&mut self, number: usize) {
        for _ in 0..number {
            let x = rand::gen_range(0.0, screen_width());
            let y = rand::gen_range(0.0, screen_height());
            self.boids.push(Boid::new(x, y));
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

    world.spawn_boids(100);

    loop {
        clear_background(BLACK);
        
        world.handle_click();
        world.update();
        world.draw();

        next_frame().await;
    }
}