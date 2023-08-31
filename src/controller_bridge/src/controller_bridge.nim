import chronos
import rclnim
import rclnim/chronossupport
import chronicles
import websock/websock
import std/uri


type
  ControllerBridge = ref object
    node: Node
    sendQueue: AsyncQueue[seq[byte]]
    recvQueue: AsyncQueue[seq[byte]]


using self: ControllerBridge

proc handle(self; request: HttpRequest) {.async.} =
  trace "Handling request:", uri = request.uri.path

  try:
    let server = WSServer.new()
    let ws = await server.handleRequest(request)
    if ws.readyState != Open:
      error "Failed to open websocket connection"
      return

    trace "Websocket handshake completed"

    proc sendTask() {.async.} =
      while ws.readyState != ReadyState.Closed:
        let data = await self.recvQueue.get()
        await ws.send(data, Opcode.Binary)

    proc recvTask() {.async.} =
      let data = await ws.recvMsg()
      await self.recvQueue.put(data)

    discard await one [sendTask(), recvTask()]

  except WebSocketError as exc:
    error "WebSocket error:", exception = exc.msg


proc main() {.async.} =
  let
    socketFlags = {ServerFlags.TcpNoDelay, ServerFlags.ReuseAddr}
    server = HttpServer.create(initTAddress("127.0.0.1:8888"), flags = socketFlags)

  let node = ControllerBridge()

  server.handler =
    proc(req: HttpRequest): Future[void] =
      handle(node, req)
  server.start()

  trace "Server listening on ", data = $server.localAddress()
  await server.join()

waitFor main()

