from setuptools import setup
package_name = 'speech_control'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VotreNom',
    maintainer_email='votre-email@example.com',
    description='Contrôle vocal pour la gestion des mots de passe',
    license='Licence',
    entry_points={
        'console_scripts': [
            'speech_recognizer = speech_control.speech_recognizer:main',
            'speech_executor = speech_control.speech_executor:main',
        ],
    },
)
