#include "selfdrive/ui/soundd/sound.h"

#include <cmath>

#include <QAudio>
#include <QAudioDeviceInfo>
#include <QDebug>

#include "common/params.h"
#include "cereal/messaging/messaging.h"
#include "common/util.h"

// TODO: detect when we can't play sounds
// TODO: detect when we can't display the UI

Sound::Sound(QObject *parent) : sm({"controlsState", "deviceState", "microphone", "carState"}) {
  qInfo() << "default audio device: " << QAudioDeviceInfo::defaultOutputDevice().deviceName();

  for (auto &[alert, fn, loops] : sound_list) {
    QSoundEffect *s = new QSoundEffect(this);
    QObject::connect(s, &QSoundEffect::statusChanged, [=]() {
      assert(s->status() != QSoundEffect::Error);
    });
    s->setSource(QUrl::fromLocalFile("../../assets/sounds/" + fn));
    sounds[alert] = {s, loops};
  }

  QTimer *timer = new QTimer(this);
  QObject::connect(timer, &QTimer::timeout, this, &Sound::update);
  timer->start(1000 / UI_FREQ);
};

void Sound::update() {
  const bool started_prev = sm["deviceState"].getDeviceState().getStarted();
  sm.update(0);

  const bool started = sm["deviceState"].getDeviceState().getStarted();
  if (started && !started_prev) {
    started_frame = sm.frame;
  }

  // no sounds while offroad
  // also no sounds if nothing is alive in case thermald crashes while offroad
  const bool crashed = (sm.frame - std::max(sm.rcv_frame("deviceState"), sm.rcv_frame("controlsState"))) > 10*UI_FREQ;
  if (!started || crashed) {
    setAlert({});
    return;
  }

  #ifdef QCOM
  if (sm.updated("carState")) {
    float volume = util::map_val(sm["carState"].getCarState().getVEgo(), 11.f, 20.f, 0.f, 1.0f);
    volume = QAudio::convertVolume(volume, QAudio::LogarithmicVolumeScale, QAudio::LinearVolumeScale);
    volume = util::map_val(volume, 0.f, 1.f, Hardware::MIN_VOLUME, Hardware::MAX_VOLUME);
    for (auto &[s, loops] : sounds) {
      s->setVolume(std::round(100 * volume) / 100);
    }
  }
  #else
  // scale volume using ambient noise level
  if (sm.updated("microphone")) {
    float volume = util::map_val(sm["microphone"].getMicrophone().getFilteredSoundPressureWeightedDb(), 30.f, 60.f, 0.f, 1.f);
    volume = QAudio::convertVolume(volume, QAudio::LogarithmicVolumeScale, QAudio::LinearVolumeScale);
    Hardware::set_volume(volume);
  }
  #endif
  setAlert(Alert::get(sm, started_frame));
}

void Sound::setAlert(const Alert &alert) {
  if (!current_alert.equal(alert)) {
    current_alert = alert;
    // stop sounds
    for (auto &[s, loops] : sounds) {
      // Only stop repeating sounds
      if (s->loopsRemaining() > 1 || s->loopsRemaining() == QSoundEffect::Infinite) {
        s->stop();
      }
    }

    // play sound
    if (shouldPlaySound(alert)) {
      auto &[s, loops] = sounds[alert.sound];
      s->setLoopCount(loops);
      s->play();
    }
  }
}

bool Sound::shouldPlaySound(const Alert &alert) {
  bool isQuietDrive = Params().getBool("QuietDrive");
    return (alert.sound == AudibleAlert::WARNING_SOFT || alert.sound == AudibleAlert::WARNING_IMMEDIATE) ||
      (!isQuietDrive && alert.sound != AudibleAlert::NONE);
}