use anyhow::{anyhow, Context, Result};
use serde::{Deserialize, Serialize};
use tokio_tungstenite::tungstenite;

use super::simulation::Simulation;

use futures_util::{SinkExt, StreamExt};
use log::{error, info};

use tokio::net::{TcpListener, TcpStream, ToSocketAddrs};

pub struct SimulationServer {
    listener: TcpListener,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommandType {
    Init,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Command {
    #[serde(rename = "type")]
    command_type: CommandType,
}

impl SimulationServer {
    pub async fn bind<A: ToSocketAddrs>(addr: A) -> Result<SimulationServer> {
        let listener = TcpListener::bind(addr).await?;

        return Ok(SimulationServer { listener });
    }

    pub async fn listen(mut self) -> Result<()> {
        let simulation = Simulation::new();

        // build specification of robots to be send to new sessions
        // NOTE: currently using the intial state of `Robot` later on we might want to use official
        // specs to be send to the client
        let specs = serde_json::to_string(simulation.robot()).context("could not parse robot")?;

        info!("Listening on: {:?}", self.listener.local_addr()?);
        tokio::spawn(async move {
            // only accept a single session
            while let Ok((mut stream, _)) = self.listener.accept().await {
                if let Err(e) = self.session(&mut stream, &specs).await {
                    error!("Closing session to {:?}", stream.local_addr());
                    error!("{e}")
                }
            }
        });

        // simulation should be always be running as when this is connected to a real robot it
        // should keep its physical sim in check
        let mut simulation = Simulation::new();
        simulation.run(|s| {});

        Ok(())
    }

    async fn session(&mut self, stream: &mut TcpStream, specs: &String) -> Result<()> {
        let addr = stream
            .peer_addr()
            .context("connected streams should have a peer address")?;
        info!("Peer address: {}", addr);

        let ws_stream = tokio_tungstenite::accept_async(stream)
            .await
            .context("Error during the websocket handshake occurred")?;
        info!("New WebSocket connection: {}", addr);

        let (mut write, mut read) = ws_stream.split();

        let command = read.next().await.context("timeout")?.context("timeout")?;

        let command: Command = match command {
            tungstenite::Message::Text(data) => {
                serde_json::from_str(&data).context("failed to parse json")?
            }
            _ => return Err(anyhow!("invalid data type")),
        };

        match command.command_type {
            CommandType::Init => {
                write
                    .send(tungstenite::Message::Text(specs.to_string()))
                    .await
                    .context("failed to send specs")?;
            }
        }

        // We should not forward messages other than text or binary.

        Ok(())
    }
}
