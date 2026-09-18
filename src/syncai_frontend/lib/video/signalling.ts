/**
 * The two pieces of WHEP and WHIP that are the same in both directions.
 *
 * The robot's worker is single-shot in either direction: POST an offer, get a
 * complete answer, and that is the entire negotiation. Both clients therefore
 * need the same two things — a complete offer before they send, and the
 * session URL out of the response — and getting either wrong fails in a way
 * that looks like something else entirely.
 */

/**
 * How long to wait for ICE gathering before sending the offer anyway.
 *
 * On a LAN with host candidates only this completes in tens of milliseconds.
 * The cap is for the case that actually stalls: STUN_SERVERS pointing at
 * Google's public server on a robot with no internet, where gathering sits
 * waiting for a reply that never comes. Sending host candidates alone is
 * correct on the LAN this runs on, so a timeout here degrades to "works
 * locally" rather than "never connects".
 */
export const ICE_GATHERING_TIMEOUT_MS = 3000;

/**
 * Resolve with the complete local description, or after the timeout with
 * whatever has been gathered so far.
 *
 * There is no candidate endpoint to PATCH to and no renegotiation, so the
 * offer we send has to already carry every candidate the browser is going to
 * find — send early and the offer goes out with no candidates at all and the
 * connection never forms.
 */
export function waitForIceGathering(pc: RTCPeerConnection): Promise<boolean> {
  if (pc.iceGatheringState === "complete") return Promise.resolve(true);

  return new Promise((resolve) => {
    let settled = false;
    const finish = (complete: boolean) => {
      if (settled) return;
      settled = true;
      pc.removeEventListener("icegatheringstatechange", onChange);
      clearTimeout(timer);
      resolve(complete);
    };
    const onChange = () => {
      if (pc.iceGatheringState === "complete") finish(true);
    };

    pc.addEventListener("icegatheringstatechange", onChange);
    const timer = setTimeout(() => finish(false), ICE_GATHERING_TIMEOUT_MS);
  });
}

/**
 * Resolve the Location header against the request URL.
 *
 * The backend answers a relative URI on purpose (it cannot know which host the
 * console reached it on), so it has to be resolved here. A null return is the
 * CORS symptom worth recognising: the header is on the wire but the browser
 * withholds it from JavaScript unless the server sends
 * Access-Control-Expose-Headers: Location. The stream then works perfectly and
 * teardown silently never happens.
 */
export function resolveSessionUrl(response: Response, requestUrl: string): string | null {
  const location = response.headers.get("Location");
  if (!location) return null;
  try {
    return new URL(location, response.url || requestUrl).toString();
  } catch {
    return null;
  }
}
