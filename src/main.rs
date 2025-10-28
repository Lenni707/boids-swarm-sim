use macroquad::prelude::*;

#[macroquad::main("Swarm Sim")]
async fn main() {
    loop {
        clear_background(BLACK);

        next_frame().await;
    }
    
}
