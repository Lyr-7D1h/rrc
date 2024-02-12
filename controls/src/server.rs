use std::{fmt::Write, ops::Deref, sync::Arc, thread::sleep, time::Duration};

use anyhow::{anyhow, Context, Result};
use crossbeam::epoch::{pin, Atomic, Owned};
use serde::{de::Error, Deserialize, Deserializer};
use tokio_tungstenite::tungstenite::{self, Message};

use crate::robot::State;

use super::simulation::Simulation;

use futures_util::{FutureExt, SinkExt, TryStreamExt};
use log::{error, info};

use tokio::{
    net::{TcpListener, TcpStream, ToSocketAddrs},
    sync::mpsc::{channel, Sender},
};

fn deserialize_specs<'de, D>(d: D) -> Result<urdf_rs::Robot, D::Error>
where
    D: Deserializer<'de>,
{
    let value: String = Deserialize::deserialize(d).unwrap();

    return urdf_rs::read_from_string(&value).map_err(|e| Error::custom(e));
}

#[derive(Debug, Deserialize)]
pub struct Limits {
    pub index: usize,
    pub acceleration: f64,
    pub velocity: f64,
    pub min: f64,
    pub max: f64,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type")]
#[serde(rename_all = "snake_case")]
pub enum Command {
    /// update current robot model
    Init {
        #[serde(deserialize_with = "deserialize_specs")]
        urdf: urdf_rs::Robot,
        limits: Vec<Limits>,
        state: State,
    },
    Move {
        // TODO: only send a single joint you want to change, otherwise it will get old state data
        state: State,
    },
    Ikmove {
        position: [f64; 3],
    },
}

pub struct SimulationServer {
    listener: TcpListener,
}

impl SimulationServer {
    pub async fn bind<A: ToSocketAddrs>(addr: A) -> Result<SimulationServer> {
        let listener = TcpListener::bind(addr).await?;

        return Ok(SimulationServer { listener });
    }

    pub async fn listen(mut self) -> Result<()> {
        let state = Arc::new(Atomic::new(vec![]));
        let (cmd_tx, cmd_rx) = channel::<Command>(100);

        info!("Listening on: {:?}", self.listener.local_addr()?);

        let state_clone = state.clone();

        // only accept a single session
        tokio::spawn(async move {
            while let Ok((mut stream, _)) = self.listener.accept().await {
                if let Err(e) = self
                    .session(&mut stream, state_clone.clone(), cmd_tx.clone())
                    .await
                {
                    error!("Closing session to {:?}", stream.local_addr());
                    error!("{e}")
                }
            }
        });

        // simulation should be always be running as when this is connected to a real robot it
        // should keep its physical sim in check and handle possible feedback
        let mut simulation = Simulation::new()?;
        simulation
            .run(cmd_rx, move |s| {
                if let Some(update) = s.robot_state() {
                    let update = Owned::new(update.clone());

                    let guard = pin();
                    let old = state.swap(update, std::sync::atomic::Ordering::SeqCst, &guard);
                    // deallocate old state
                    unsafe { guard.defer_destroy(old) }
                }
            })
            .await;

        Ok(())
    }

    async fn session(
        &mut self,
        stream: &mut TcpStream,
        state: Arc<Atomic<State>>,
        cmd_tx: Sender<Command>,
    ) -> Result<()> {
        let addr = stream
            .peer_addr()
            .context("connected streams should have a peer address")?;
        info!("Peer address: {}", addr);

        let mut ws_stream = tokio_tungstenite::accept_async(stream)
            .await
            .context("Error during the websocket handshake occurred")?
            .into_stream();
        info!("New WebSocket connection: {}", addr);

        loop {
            // parse and forward commands from stream to simulation
            if let Some(msg) = ws_stream.try_next().now_or_never() {
                if let Some(message) = msg.context("error occurred while reading from stream")? {
                    let command: Command = match message {
                        Message::Text(text) => {
                            serde_json::from_str(&text).context("failed to parse command")?
                        }
                        Message::Close(_) => return Ok(()),
                        _ => return Err(anyhow!("invalid message data type")),
                    };
                    cmd_tx
                        .send(command)
                        .await
                        .context("failed to send command to simulation")?;
                }
            };

            let update = {
                let p = pin();
                let update = unsafe {
                    state
                        .load(std::sync::atomic::Ordering::Relaxed, &p)
                        .deref()
                        .deref()
                };
                serde_json::to_string(update).context("failed to parse joint values")?
            };
            ws_stream
                .send(tungstenite::Message::Text(update))
                .await
                .context("failed to send update")?;

            // Wait ~1/60 second before sending new update
            sleep(Duration::from_millis(16));
        }
    }
}
