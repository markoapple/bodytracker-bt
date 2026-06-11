export class UIStore {
  constructor() {
    this.state = {
      backend: {},
      config: {},
      runtime: { running: false, last_error: null },
      debug: {},
      calibration: {},
      floorGeometry: { auto: null, draft: null, cleared: false },
      metricHistory: {},
      dirty: false
    };
  }

  getState() { return this.state; }

  dispatch(action) {
    switch (action?.type) {
      case "STATE_RECEIVED": {
        const backend = action.state || {};
        this.state = Object.assign({}, this.state, {
          backend,
          config: backend.config || {},
          runtime: backend.runtime || {},
          debug: backend.debug || {},
          calibration: backend.calibration || {}
        });
        break;
      }
      case "DRAFT_DIRTY":
        this.state = Object.assign({}, this.state, { dirty: true });
        break;
      case "DRAFT_CLEAN":
        this.state = Object.assign({}, this.state, { dirty: false });
        break;
      case "FLOOR_GEOMETRY":
        this.state = Object.assign({}, this.state, {
          floorGeometry: Object.assign({}, this.state.floorGeometry || {}, action.patch || {})
        });
        break;
      case "METRIC_PUSH": {
        const key = String(action.key || "unknown");
        const value = Number(action.value);
        if (!Number.isFinite(value)) break;
        const metricHistory = Object.assign({}, this.state.metricHistory || {});
        const values = Array.isArray(metricHistory[key]) ? metricHistory[key].slice() : [];
        values.push(value);
        while (values.length > 240) values.shift();
        metricHistory[key] = values;
        this.state = Object.assign({}, this.state, { metricHistory });
        break;
      }
      case "PENDING_COMMANDS":
        this.state = Object.assign({}, this.state, { pendingCommands: action.payload || {} });
        break;
      default:
        break;
    }
    return this.state;
  }
}

export const configEventStore = (() => {
  const key = "bodytracker.config.events.v1";
  const maxEvents = 300;
  let events = [];
  try {
    const parsed = JSON.parse(window.localStorage?.getItem(key) || "[]");
    events = Array.isArray(parsed) ? parsed : [];
  } catch (_) {
    events = [];
  }
  function persist() {
    try { window.localStorage?.setItem(key, JSON.stringify(events.slice(-maxEvents))); } catch (_) {}
  }
  return {
    append(type, payload = {}, metadata = {}) {
      const event = {
        id: `${Date.now()}-${Math.random().toString(36).slice(2)}`,
        timestamp: Date.now(),
        type,
        payload,
        metadata
      };
      events.push(event);
      while (events.length > maxEvents) events.shift();
      persist();
      return event;
    }
  };
})();
