#!/usr/bin/env python
from gtts import gTTS
import os
import pygame


class Talker:

    def __init__(self, text):
        
        self.clothes_response = {
            "cap" : "This person is wearing a cap.",
            "glasses" : "This person is wearing glasses.",
            "red_t_shirt" : "This person is wearing a red T-Shirt."
        }

        self.text = self.text_init(text)
        # Create a text-to-speech object and specify the language
        tts = gTTS(text=self.text, lang='en')
        self.audio_file = '/home/mustar/jupiter/matus/matus_showcase/tmp_audio/response.mp3'

        # Save the audio file
        tts.save(self.audio_file)

    def talk(self):
        pygame.mixer.init()
        pygame.mixer.music.load(self.audio_file)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            continue

        pygame.mixer.quit()
        self.delete_audio_file()

    # Delete the audio file
    def delete_audio_file(self):
        try:
            os.remove(self.audio_file)
            print("Temporary audio file deleted successfully")
        except OSError as e:
            print("Error deleting file:", e)

    def text_init(self, text):
        if text in self.clothes_response:
            return self.clothes_response[text]
        return text
    