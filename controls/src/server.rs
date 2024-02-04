use std::{ops::Deref, sync::Arc, thread::sleep, time::Duration};

use anyhow::{anyhow, Context, Result};
use crossbeam::epoch::{pin, Atomic, Owned};
use serde::{Deserialize, Serialize};
use tokio_tungstenite::tungstenite::{self, Message};

use crate::{simulation::JointValue, specs::Specs};

use super::simulation::Simulation;

use futures_util::{FutureExt, SinkExt, TryStreamExt};
use log::{error, info};

use tokio::{
    net::{TcpListener, TcpStream, ToSocketAddrs},
    sync::mpsc::{channel, Sender},
};

// pub struct State<N> {
//     ro: Arc<N>,
//     has_next: AtomicBool,
//     next: Option<Arc<State<N>>>,
// }
//
// impl<N> State<N> {
//     pub fn new(value: N) -> Arc<State<N>> {
//         return Arc::new(State {
//             ro: Arc::new(value),
//             has_next: AtomicBool::new(false),
//             next: None,
//         });
//     }
//
//     pub fn clone(state: &Arc<State<N>>) -> Arc<State<N>> {
//         state.clone()
//     }
//
//     pub fn update(&mut self, value: N) {
//         self.next = Some(State::new(value));
//         self.has_next
//             .store(true, std::sync::atomic::Ordering::Relaxed)
//     }
// }
//
// impl<N> DerefMut for State<N> {
//     fn deref_mut(&mut self) -> &mut Self::Target {
//         todo!()
//     }
// }
//
// pub fn read<N>(mut state: Arc<State<N>>) -> Arc<N> {
//     while state.has_next.load(std::sync::atomic::Ordering::Relaxed) {
//         state = state.next.expect("expected next state, has_next flag set");
//     }
//     state.ro
// }
//
#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "type")]
#[serde(rename_all = "snake_case")]
pub enum Command {
    Init { specs: Specs },
    Move { position: [f32; 3] },
}

pub struct SimulationServer {
    listener: TcpListener,
}

type State = Arc<Atomic<Owned<Vec<JointValue>>>>;
impl SimulationServer {
    pub async fn bind<A: ToSocketAddrs>(addr: A) -> Result<SimulationServer> {
        let listener = TcpListener::bind(addr).await?;

        return Ok(SimulationServer { listener });
    }

    pub fn listen(mut self) -> Result<()> {
        let state = Arc::new(Atomic::new(Owned::<Vec<JointValue>>::new(vec![])));
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
        let mut simulation = Simulation::new();
        simulation.run(cmd_rx, move |s| {
            let update = s.robot().joint_values();
            let update = Owned::new(update);

            state.store(Owned::new(update), std::sync::atomic::Ordering::Relaxed);
        });

        Ok(())
    }

    async fn session(
        &mut self,
        stream: &mut TcpStream,
        state: State,
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
            if let Some(msg) = ws_stream.try_next().now_or_never() {
                if let Some(message) = msg.context("error occurred while reading from stream")? {
                    let command = match message {
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

            // TODO ensure update frequency makes for smooth graphical display
            // Wait ~1/60 second before sending new update
            sleep(Duration::from_millis(16));
        }
    }
}
