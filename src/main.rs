use macroquad::prelude::*;
use macroquad::rand::gen_range;

// -!TODO!- 
// - so ein cone für die sicht des boids adden damit er z.b nicht hinter sich gucken kann
// raubvogel adden vor dem die boids wegfliegen
// wind adden was die beeinflusst
// prefrences: manches boids sind eher links/rechts zentriert
// 3d machen

const SCREEN_WIDTH: i32 = 1400;
const SCREEN_HEIGHT: i32 = 800;

const VISUAL_RANGE: f32 = 75.0;           // sight distance
const COHERENCE: f32 = 0.002 * 0.4;             // how much boids move toward group center
const AVOIDFACTOR: f32 = 0.05 * 2.0;            // how strongly they avoid others // war mal 0.08
const AVOIDDISTANCE: f32 = 20.0;          // minimum allowed distance between boids
const ALIGNMENTFACTOR: f32 = 0.05 * 0.5;        // how strongly they match neighbor velocity

const TURNFACTOR: f32 = 0.2;              // how fast they turn near edges
const EDGE_DISTANCE: f32 = 100.0;         // how close to edge before turning

const MAX_SPEED: f32 = 6.0;
const MIN_SPEED: f32 = 3.0;
const MAX_TURN: f32 = 3.0;


#[derive(PartialEq)]
struct Boid {
    pos: Vec2,
    vel: Vec2,
}

impl Boid {
    fn new(x: f32, y: f32) -> Self {
        let vel = Vec2::new(
            rand::gen_range(-5.0, 5.0),
            rand::gen_range(-5.0, 5.0),
        );
        Self {
            pos: Vec2::new(x, y),
            vel,
        }
    }
    fn draw(&self) {
        let angle = self.vel.y.atan2(self.vel.x);
        let size = 6.0;
        let p1 = self.pos + Vec2::from_angle(angle) * size;
        let p2 = self.pos + Vec2::from_angle(angle + 2.5) * size * 0.6;
        let p3 = self.pos + Vec2::from_angle(angle - 2.5) * size * 0.6;
        draw_triangle(p1, p2, p3, WHITE);
    }
}

struct World {
    boids: Vec<Boid>,
}

impl World {
    fn new() -> Self {
        Self { boids: vec![] }
    }

    fn handle_click(&mut self) {
        if is_mouse_button_pressed(MouseButton::Left) {
            let (x, y) = mouse_position();
            self.boids.push(Boid::new(x, y));
        }
    }

    fn draw(&self) {
        for boid in &self.boids {
            boid.draw(); // method defined in impl Boid
        }
    }

    fn alignment(&self) -> Vec<Vec2> { // wie doll die boids den speed von den anderen matchen wollen
        let mut changed_vels = vec![];
        for self_boid in &self.boids {
            let mut boids_vel = Vec2::ZERO;
            let mut neighbors = 0.0;
            for other_boid in &self.boids {
                if self_boid == other_boid {
                    continue;
                }
                let distance = self_boid.pos.distance(other_boid.pos);
                if distance < VISUAL_RANGE { // alle boids in der range
                    boids_vel += other_boid.vel;
                    neighbors += 1.0;
                }
            }
            if neighbors > 0.0 {
                let avg_vel = boids_vel / neighbors;
                let adjustment = (avg_vel - self_boid.vel) * ALIGNMENTFACTOR;
                changed_vels.push(adjustment);
            } else {
                changed_vels.push(Vec2::ZERO);
            }
        }
        changed_vels
    }

    fn separation(&self) -> Vec<Vec2> { // abstand zwischen den boids
        let mut changed_vels = vec![];
        for self_boid in &self.boids {
            let mut move_away = Vec2::ZERO;
            for other_boid in &self.boids {
                if self_boid == other_boid {
                    continue;
                }
                let distance = self_boid.pos.distance(other_boid.pos);
                if distance < AVOIDDISTANCE && distance > 0.0 {
                    let diff = self_boid.pos - other_boid.pos;
                    move_away += diff / distance;
                }
            }
            move_away *= AVOIDFACTOR;
            changed_vels.push(move_away);
        }
        changed_vels
    }

    fn cohesion(&self) -> Vec<Vec2> { // wie doll die boids zur durchschnitts pos der anderen fliegen will
        let mut changed_vels = vec![];
        for self_boid in &self.boids {
            let mut center = Vec2::ZERO;
            let mut neighbors = 0.0;
            for other_boid in &self.boids {
                if self_boid == other_boid {
                    continue;
                }
                let distance = self_boid.pos.distance(other_boid.pos);
                if distance < VISUAL_RANGE {
                    center += other_boid.pos;
                    neighbors += 1.0;
                }
            }
            if neighbors > 0.0 {
                let avg_pos = center / neighbors;
                let adjustment = (avg_pos - self_boid.pos) * COHERENCE;
                changed_vels.push(adjustment);
            } else {
                changed_vels.push(Vec2::ZERO);
            }
        }
        changed_vels
    }

    fn update_velocities(&mut self) -> Vec<Vec2> {
        if self.boids.is_empty() {
            return vec![];
        }

        let alignment = self.alignment();
        let separation = self.separation();
        let cohesion = self.cohesion();


        let edge_avoid = check_edge_boids(&self.boids);

        let mut final_vels = vec![Vec2::ZERO; self.boids.len()]; // das letzere muss iwe weil dann die größe des vecs bekannt ist

        for i in 0..self.boids.len() { // alles addieren
            final_vels[i] =
                alignment[i] +
                separation[i] +
                cohesion[i] +
                edge_avoid[i] * 1.0;
        }

        final_vels
    }

    fn update(&mut self) {
        if self.boids.is_empty() {
            return;
        }

        let updated_vels = self.update_velocities();

        for i in 0..self.boids.len() {
            let old_vel = self.boids[i].vel;
            self.boids[i].vel += updated_vels[i];

            // bisschen randomniss
            self.boids[i].vel += Vec2::new(gen_range(-0.1, 0.1), gen_range(-0.1, 0.1));

            // tempolimit damit die werte nicht unednlich groß werden können
            let speed = self.boids[i].vel.length();
            if speed > MAX_SPEED {
                self.boids[i].vel = self.boids[i].vel.normalize() * MAX_SPEED;
            }
            if speed < MIN_SPEED {
                self.boids[i].vel = self.boids[i].vel.normalize() * MIN_SPEED;
            }

            // wie doll sich der vector veräändern darf
            let vel_change = self.boids[i].vel - old_vel;
            if vel_change.length() > MAX_TURN {
                self.boids[i].vel = old_vel + vel_change.normalize() * MAX_TURN;
            }

            // Move boid
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

// pub functions

// wandelt boids.pos in eine vec2 um
fn check_edge_boids(boids: &[Boid]) -> Vec<Vec2> {
    check_edge_positions(&boids.iter().map(|b| b.pos).collect::<Vec<_>>())
}

// macht die math und nimmt nur vec2 als parameter deswegen brauchen wir die obige umwandlungsfunktion
fn check_edge_positions(positions: &[Vec2]) -> Vec<Vec2> {
    let mut adjustments = Vec::with_capacity(positions.len());
    for pos in positions {
        let mut move_away = Vec2::ZERO;

        if pos.x < EDGE_DISTANCE {
            move_away.x += TURNFACTOR;
        } else if pos.x > SCREEN_WIDTH as f32 - EDGE_DISTANCE {
            move_away.x -= TURNFACTOR;
        }

        if pos.y < EDGE_DISTANCE {
            move_away.y += TURNFACTOR;
        } else if pos.y > SCREEN_HEIGHT as f32 - EDGE_DISTANCE {
            move_away.y -= TURNFACTOR;
        }

        adjustments.push(move_away);
    }
    adjustments
}


fn window_conf() -> Conf {
    Conf {
        window_title: "Boids Simulation (Macroquad)".to_string(),
        window_width: SCREEN_WIDTH,
        window_height: SCREEN_HEIGHT,
        fullscreen: false,
        ..Default::default()
    }
}
#[macroquad::main(window_conf)]
async fn main() {
    let mut world = World::new();
    world.spawn_boids(300);

    loop {
        clear_background(BLACK);

        world.handle_click();
        world.update();
        world.draw();

        next_frame().await;
    }
}