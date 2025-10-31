use macroquad::prelude::*;
use macroquad::rand::gen_range;
use std::collections::HashMap;

// -!TODO!- 
// - so ein cone für die sicht des boids adden damit er z.b nicht hinter sich gucken kann
// raubvogel adden vor dem die boids wegfliegen
// wind adden was die beeinflusst
// prefrences: manches boids sind eher links/rechts zentriert
// 3d machen

const SCREEN_WIDTH: i32 = 1400;
const SCREEN_HEIGHT: i32 = 800;

const VISUAL_RANGE: f32 = 75.0;           // sight distance
const VISUAL_RANGE_SQ: f32 = VISUAL_RANGE * VISUAL_RANGE;  // für distance_squared optimization , weiß auch nicht warum danke claude <3

const SEE_PRED_RANGE: f32 = 100.0;

const COHERENCE: f32 = 0.002 * 0.35;             // how much boids move toward group center

const AVOIDFACTOR: f32 = 0.05 * 2.0;            // how strongly they avoid others // war mal 0.08
const AVOIDDISTANCE: f32 = 20.0;          // minimum allowed distance between boids
const AVOIDDISTANCE_SQ: f32 = AVOIDDISTANCE * AVOIDDISTANCE;  // für distance_squared optimization, weiß auch nicht warum danke claude <3

const ALIGNMENTFACTOR: f32 = 0.05 * 0.5;        // how strongly they match neighbor velocity

const FLEE_FACTOR: f32 = 0.01;             // how much they wanna flee from a pred

const TURNFACTOR: f32 = 0.2;              // how fast they turn near edges
const EDGE_DISTANCE: f32 = 100.0;         // how close to edge before turning

const MAX_SPEED: f32 = 5.0;
const MIN_SPEED: f32 = 2.5;
const MAX_TURN: f32 = 3.0;

const CELL_SIZE: f32 = 75.0;              // spatial partitioning cell size (same as VISUAL_RANGE)

const PRED_VISUAL_RANGE: f32 = 200.0;
const PRED_MAX_SPEED: f32 = 6.0;
const PRED_MIN_SPEED: f32 = 3.0;
const PRED_MAX_TURN: f32 = 2.5;


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

struct Pred {
    pos: Vec2,
    vel: Vec2
}

impl Pred {
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
        let size = 8.0;
        let p1 = self.pos + Vec2::from_angle(angle) * size;
        let p2 = self.pos + Vec2::from_angle(angle + 2.5) * size * 0.6;
        let p3 = self.pos + Vec2::from_angle(angle - 2.5) * size * 0.6;
        draw_triangle(p1, p2, p3, RED);
    }
}

// Spatial partitioning grid, iwie efficienter
struct SpatialGrid {
    cells: HashMap<(i32, i32), Vec<usize>>,
}

impl SpatialGrid {
    fn new() -> Self {
        Self {
            cells: HashMap::new(),
        }
    }

    // gibt die cell koordinaten für eine position zurück
    fn get_cell_coords(&self, pos: Vec2) -> (i32, i32) {
        ((pos.x / CELL_SIZE) as i32, (pos.y / CELL_SIZE) as i32)
    }

    // baut das grid neu auf basierend auf aktuellen boid positionen
    fn rebuild(&mut self, boids: &[Boid]) {
        self.cells.clear();
        for (i, boid) in boids.iter().enumerate() {
            let cell = self.get_cell_coords(boid.pos);
            self.cells.entry(cell).or_insert_with(Vec::new).push(i);
        }
    }

    // gibt alle nachbar boids zurück (9 cells: current + 8 umliegende)
    fn get_neighbors(&self, pos: Vec2) -> Vec<usize> {
        let (cx, cy) = self.get_cell_coords(pos);
        let mut neighbors = Vec::new();

        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(indices) = self.cells.get(&(cx + dx, cy + dy)) {
                    neighbors.extend(indices);
                }
            }
        }
        neighbors
    }
}

struct World {
    boids: Vec<Boid>,
    preds: Vec<Pred>,
    grid: SpatialGrid,              // das Grid
    velocity_buffer: Vec<Vec2>,     // wiederverwendbarer zwischenspeicher für die updates der velocity
    mouse_chase: bool
}

impl World {
    fn new() -> Self {
        Self {
            boids: vec![],
            preds: vec![],
            grid: SpatialGrid::new(),
            velocity_buffer: vec![],
            mouse_chase: false
        }
    }
    // spawns a pred
    fn handle_click(&mut self) {
        if is_mouse_button_pressed(MouseButton::Left) {
            let (x, y) = mouse_position();
            self.preds.push(Pred::new(x, y));
        }
    }

    fn handle_keys(&mut self) {
        if is_key_pressed(KeyCode::R) { // clear the screen
            self.boids.clear();
            self.preds.clear();
            self.grid = SpatialGrid::new();
            self.velocity_buffer.clear();
        }
        if is_key_pressed(KeyCode::Space) {
            self.spawn_boids(50);
        }
        if is_key_pressed(KeyCode::E) {
            if self.mouse_chase == false {
                self.mouse_chase = true;
            }
            else {
                self.mouse_chase = false;
            }
        }
    }

    fn draw(&self) {
        for boid in &self.boids {
            boid.draw(); // method defined in impl Boid
        }
        for pred in &self.preds {
            pred.draw()
        }
    }

    fn draw_stats(&self) {
        let fps = get_fps();
        let boid_count = self.boids.len();
        let pred_count = self.preds.len();
        
        draw_text(&format!("FPS: {}", fps), 10.0, 20.0, 20.0, WHITE);
        draw_text(&format!("Boids: {}", boid_count), 10.0, 40.0, 20.0, WHITE);
        draw_text(&format!("Preds: {}", pred_count), 10.0, 60.0, 20.0, RED);
    }

    fn pred_chase_prey(&mut self) {
        let detection_radius_sq = PRED_VISUAL_RANGE * PRED_VISUAL_RANGE;

        // edge avoidance für preds
        let edge_avoid = check_edge_preds(&self.preds);

        for (i, pred) in self.preds.iter_mut().enumerate() {
            let mut center_of_mass = Vec2::ZERO;
            let mut count = 0;

            // find boids within detection radius and calculate their average position
            for boid in &self.boids {
                let dist_sq = (boid.pos - pred.pos).length_squared();
                if dist_sq < detection_radius_sq {
                    center_of_mass += boid.pos;
                    count += 1;
                }
            }

            let old_vel = pred.vel;
            let mut adjustment = Vec2::ZERO;

            // if boids detected, move toward their center (highest concentration)
            if count > 0 {
                center_of_mass /= count as f32;
                adjustment += (center_of_mass - pred.pos).normalize_or_zero() * 0.3;
            }

            // apply edge avoidance
            adjustment += edge_avoid[i];
            
            // bisschen randomness
            adjustment += Vec2::new(gen_range(-0.1, 0.1), gen_range(-0.1, 0.1));

            pred.vel += adjustment;

            // speed limits für preds
            let speed = pred.vel.length();
            if speed > PRED_MAX_SPEED {
                pred.vel = pred.vel.normalize() * PRED_MAX_SPEED;
            }
            if speed < PRED_MIN_SPEED {
                pred.vel = pred.vel.normalize() * PRED_MIN_SPEED;
            }

            // wie doll sich der vector verändern darf
            let vel_change = pred.vel - old_vel;
            if vel_change.length() > PRED_MAX_TURN {
                pred.vel = old_vel + vel_change.normalize() * PRED_MAX_TURN;
            }
        }
    }


    // OPTIMIERT: kombiniert alignment, separation, cohesion in einer loop
    // cached distance calculations, nutzt spatial grid für neighbor lookup
    fn calculate_flocking_forces(&mut self) {
        for (i, self_boid) in self.boids.iter().enumerate() {
            // spatial grid: nur nachbarn in nahen cells checken
            let neighbor_indices = self.grid.get_neighbors(self_boid.pos);

            let mut align_sum = Vec2::ZERO;       // wie doll die boids den speed von den anderen matchen wollen
            let mut cohesion_sum = Vec2::ZERO;    // wie doll die boids zur durchschnitts pos der anderen fliegen will
            let mut separate_vel = Vec2::ZERO;    // abstand zwischen den boids
            let mut flee_vec = Vec2::ZERO;        // wie doll sie flüchten vorm pred
            let mut mouse_flee_vel = Vec2::ZERO;
            let mut neighbor_count = 0.0;

            // nur durch spatial neighbors loopen statt alle boids
            for &neighbor_idx in &neighbor_indices {
                if i == neighbor_idx {
                    continue;
                }
                let other_boid = &self.boids[neighbor_idx];

                // einmal distance berechen -> immer wieder benutzen
                let diff = self_boid.pos - other_boid.pos;
                let dist_sq = diff.length_squared();  // distance_squared ist schneller (kein sqrt)

                // Alignment & Cohesion: alle boids in der range
                if dist_sq < VISUAL_RANGE_SQ {
                    align_sum += other_boid.vel;
                    cohesion_sum += other_boid.pos;
                    neighbor_count += 1.0;
                }

                // Separation: minimum allowed distance zwischen boids
                if dist_sq < AVOIDDISTANCE_SQ && dist_sq > 0.0 {
                    let distance = dist_sq.sqrt();
                    separate_vel += diff / distance;
                }
            }

            // boids wollen for preds fliehen
            for pred in self.preds.iter() {
                let distance = self_boid.pos.distance(pred.pos);
                if distance < SEE_PRED_RANGE {
                    flee_vec += self_boid.pos - pred.pos; // flee away from predator
                }
            }

            if self.mouse_chase == true {
                let mouse_pos = Vec2::from(mouse_position());
                let distance = self_boid.pos.distance(mouse_pos);
                if distance < VISUAL_RANGE {
                    mouse_flee_vel += self_boid.pos - mouse_pos
                }
            }

            // die veränderung speichern und immer dazu addieren
            let mut adjustment = Vec2::ZERO;

            // wenn mehr als ein nachbar da ist, die werte durch anzahl nachbarn teilen für durchschnitt
            if neighbor_count > 0.0 {
                let avg_vel = align_sum / neighbor_count;
                adjustment += (avg_vel - self_boid.vel) * ALIGNMENTFACTOR;

                let avg_pos = cohesion_sum / neighbor_count;
                adjustment += (avg_pos - self_boid.pos) * COHERENCE;
            }

            adjustment += separate_vel * AVOIDFACTOR;

            adjustment += flee_vec * FLEE_FACTOR;

            adjustment += mouse_flee_vel * FLEE_FACTOR;

            self.velocity_buffer[i] = adjustment;
        }
    }

    fn update_velocities(&mut self) {
        if self.boids.is_empty() {
            return;
        }

        // buffer größe anpassen wenn nötig (nur bei boid count änderung)
        if self.velocity_buffer.len() != self.boids.len() {
            self.velocity_buffer.resize(self.boids.len(), Vec2::ZERO);
        }

        // spatial grid neu bauen mit aktuellen positionen
        self.grid.rebuild(&self.boids);

        // flocking forces berechnen (alignment, separation, cohesion kombiniert)
        self.calculate_flocking_forces();

        // edge avoidance berechnen
        let edge_avoid = check_edge_boids(&self.boids);

        // alles kombinieren und auf boids anwenden
        for i in 0..self.boids.len() {
            let old_vel = self.boids[i].vel;

            // alles addieren
            self.boids[i].vel += self.velocity_buffer[i] + edge_avoid[i];

            // bisschen randomniss
            self.boids[i].vel += Vec2::new(gen_range(-0.1, 0.1), gen_range(-0.1, 0.1));

            // tempolimit damit die werte nicht unendlich groß werden können
            let speed = self.boids[i].vel.length();
            if speed > MAX_SPEED {
                self.boids[i].vel = self.boids[i].vel.normalize() * MAX_SPEED;
            }
            if speed < MIN_SPEED {
                self.boids[i].vel = self.boids[i].vel.normalize() * MIN_SPEED;
            }

            // wie doll sich der vector verändern darf
            let vel_change = self.boids[i].vel - old_vel;
            if vel_change.length() > MAX_TURN {
                self.boids[i].vel = old_vel + vel_change.normalize() * MAX_TURN;
            }
        }
    }

    fn update(&mut self) {
        if self.boids.is_empty() {
            return;
        }

        self.pred_chase_prey();
        self.update_velocities();

        for boid in &mut self.boids {
            boid.pos += boid.vel;
        }
        
        for pred in &mut self.preds {
            pred.pos += pred.vel;
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

fn check_edge_preds(preds: &[Pred]) -> Vec<Vec2> {
    check_edge_positions(&preds.iter().map(|p| p.pos).collect::<Vec<_>>())
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
        window_title: "Boids Simulation (Macroquad) - OPTIMIZED".to_string(),
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
        world.handle_keys();
        world.update();
        world.draw();
        world.draw_stats();

        next_frame().await;
    }
}
