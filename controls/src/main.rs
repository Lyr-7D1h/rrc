use controls::server::SimulationServer;
use env_logger::Env;

#[tokio::main]
async fn main() {
    env_logger::Builder::from_env(Env::default().default_filter_or("info")).init();

    let addr = "0.0.0.0:6543";
    SimulationServer::bind(&addr)
        .await
        .expect(&format!("failed to bind to {addr}"))
        .listen()
        .unwrap();
}
