export const initPlaybackControls = () => {
  const playbackToggle = document.getElementById("playback-toggle");
  const playbackPanel = document.getElementById("playback-panel");
  const playbackSpeedMenu = document.getElementById("playback-speed-menu");
  const playbackSpeedButton = playbackPanel
    ? playbackPanel.querySelector(".playback-btn-fast")
    : null;
  const playbackPlayButton = playbackPanel
    ? playbackPanel.querySelector(".playback-btn-play")
    : null;
  const playbackPauseButton = playbackPanel
    ? playbackPanel.querySelector(".playback-btn-pause")
    : null;
  const playbackStopButton = playbackPanel
    ? playbackPanel.querySelector(".playback-btn-stop")
    : null;
  const playbackResetButton = playbackPanel
    ? playbackPanel.querySelector(".playback-btn-reset")
    : null;
  const missionSkipButton = document.getElementById("mission-skip-start");
  const playbackSpeedOptions = playbackPanel
    ? Array.from(playbackPanel.querySelectorAll(".playback-speed-option"))
    : [];

  if (!playbackToggle || !playbackPanel) {
    return;
  }

  const setPlaybackOpen = (open) => {
    const next = Boolean(open);
    playbackPanel.classList.toggle("is-open", next);
    playbackPanel.setAttribute("aria-hidden", next ? "false" : "true");
    playbackToggle.classList.toggle("is-active", next);
  };

  const setSpeedMenuOpen = (open) => {
    if (!playbackSpeedMenu || !playbackSpeedButton) {
      return;
    }
    const next = Boolean(open);
    playbackSpeedMenu.classList.toggle("is-open", next);
    playbackSpeedMenu.setAttribute("aria-hidden", next ? "false" : "true");
    playbackSpeedButton.setAttribute("aria-expanded", next ? "true" : "false");
  };

  const setPlaying = (playing) => {
    if (playbackPlayButton) {
      playbackPlayButton.classList.toggle("is-active", Boolean(playing));
    }
  };

  const stopPlayback = () => {
    setPlaying(false);
  };

  const getSimClient = () => window.simClient || null;

  const setSpeedFromOption = (option) => {
    const speed = Number(option?.dataset?.speed);
    if (!Number.isFinite(speed)) {
      return;
    }
    const sim = getSimClient();
    if (sim && typeof sim.setSpeed === "function") {
      sim.setSpeed(speed);
    }
  };

  playbackToggle.addEventListener("click", (event) => {
    event.stopPropagation();
    const next = !playbackPanel.classList.contains("is-open");
    setPlaybackOpen(next);
    if (!next) {
      setSpeedMenuOpen(false);
    }
  });

  playbackPanel.addEventListener("click", (event) => {
    event.stopPropagation();
  });

  if (playbackSpeedButton && playbackSpeedMenu) {
    playbackSpeedButton.addEventListener("click", (event) => {
      event.stopPropagation();
      const next = !playbackSpeedMenu.classList.contains("is-open");
      setSpeedMenuOpen(next);
    });
  }

  playbackSpeedOptions.forEach((option) => {
    option.addEventListener("click", (event) => {
      event.stopPropagation();
      playbackSpeedOptions.forEach((item) => item.classList.remove("is-active"));
      option.classList.add("is-active");
      setSpeedMenuOpen(false);
      setSpeedFromOption(option);
    });
  });

  if (playbackSpeedOptions.length > 0) {
    playbackSpeedOptions[0].classList.add("is-active");
    setSpeedFromOption(playbackSpeedOptions[0]);
  }

  if (playbackPlayButton) {
    playbackPlayButton.addEventListener("click", async (event) => {
      event.stopPropagation();
      const next = !playbackPlayButton.classList.contains("is-active");
      setPlaying(next);
      const sim = getSimClient();
      if (sim) {
        if (next && typeof sim.play === "function") {
          await sim.play();
        } else if (!next && typeof sim.pause === "function") {
          await sim.pause();
        }
      }
    });
  }

  if (playbackPauseButton) {
    playbackPauseButton.addEventListener("click", async (event) => {
      event.stopPropagation();
      stopPlayback();
      const sim = getSimClient();
      if (sim && typeof sim.pause === "function") {
        await sim.pause();
      }
    });
  }

  if (playbackStopButton) {
    playbackStopButton.addEventListener("click", async (event) => {
      event.stopPropagation();
      stopPlayback();
      const sim = getSimClient();
      if (sim && typeof sim.stop === "function") {
        await sim.stop();
      }
    });
  }

  if (playbackResetButton) {
    playbackResetButton.addEventListener("click", async (event) => {
      event.stopPropagation();
      stopPlayback();
      if (playbackSpeedOptions.length > 0) {
        playbackSpeedOptions.forEach((item) => item.classList.remove("is-active"));
        playbackSpeedOptions[0].classList.add("is-active");
        setSpeedFromOption(playbackSpeedOptions[0]);
      }
      const sim = getSimClient();
      if (sim && typeof sim.clear === "function") {
        await sim.clear();
      } else if (sim && typeof sim.reset === "function") {
        await sim.reset();
      }
      if (sim && typeof sim.resetIntegration === "function") {
        await sim.resetIntegration();
      }
      if (typeof window.clearMissionData === "function") {
        window.clearMissionData();
      }
    });
  }

  if (missionSkipButton) {
    missionSkipButton.addEventListener("click", async (event) => {
      event.stopPropagation();
      if (missionSkipButton.disabled) {
        return;
      }
      const sim = getSimClient();
      if (!sim || typeof sim.skipToMissionStart !== "function") {
        return;
      }
      missionSkipButton.disabled = true;
      missionSkipButton.classList.add("is-busy");
      try {
        await sim.skipToMissionStart();
      } finally {
        missionSkipButton.classList.remove("is-busy");
        const state = typeof sim.getState === "function" ? sim.getState() : null;
        missionSkipButton.disabled = !state?.vehicles || Object.keys(state.vehicles).length === 0;
      }
    });
  }

  const sim = getSimClient();
  if (sim && typeof sim.subscribe === "function") {
    sim.subscribe((state) => {
      if (!state) {
        return;
      }
      const playing = Boolean(state.running && !state.paused);
      setPlaying(playing);
      if (missionSkipButton && !missionSkipButton.classList.contains("is-busy")) {
        missionSkipButton.disabled = !state.vehicles || Object.keys(state.vehicles).length === 0;
      }
      const speed = Number(state.speedFactor);
      if (Number.isFinite(speed) && playbackSpeedOptions.length > 0) {
        playbackSpeedOptions.forEach((item) => {
          const itemSpeed = Number(item.dataset.speed);
          item.classList.toggle("is-active", itemSpeed === speed);
        });
      }
    });
  }

  document.addEventListener("click", (event) => {
    const target = event.target;
    if (!playbackPanel.contains(target) && target !== playbackToggle) {
      setPlaybackOpen(false);
    }
    if (playbackSpeedMenu && playbackSpeedButton) {
      if (!playbackSpeedMenu.contains(target) && target !== playbackSpeedButton) {
        setSpeedMenuOpen(false);
      }
    }
  });
};
