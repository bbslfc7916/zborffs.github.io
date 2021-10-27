# Lord of the Rings: Chess

*<p style="text-align: center;">August 30, 2021</p>*

## Project Objective
The purpose of this project was to learn [SFML](https://github.com/SFML/sfml) and [Dear ImGui](https://github.com/ocornut/imgui) by making a Chess UI to play against [UCI-compliant](https://www.chessprogramming.org/UCI) chess engines. For added fun, I made the UI _Lord of the Rings_ themed. 

The main challenge of this project was to develop something that is at once __maintainable__ (in the sense that, a programmer could easily understand the code and add new features) but __efficient__ (in the sense that my computer's fan doesn't turn on whenever I use the app). 

The game a still a work in-progress. 

### Demo
<video width="700" controls autoplay>
    <source src="lotr-chess-images/lotrchess_demo.mp4" type="video/mp4" playbackRate=2.0>
</video>

*<p style="text-align: center;">Figure 1: Demo video of me playing the game multiple times.</p>*

### Download / Install
As of right now, the only way of installing the app is from source. So go to the [source repo](https://github.com/zborffs/lotrchess) and follow the build instructions. 

It should take roughly 5 minutes to download and install the app, assuming you already have the pre-requisite programs for the installation (CMake and conan).

## SFML + Dear ImGui
SFML and Dear ImGui are two C++ libraries (with support for other languages) that are often deployed jointly for simple 2-D graphics applications. 

The core of SFML apps is the game-loop. The game-loop is responsible for updating the game state from user-inputs and rendering the game-state onto the screen.

```Cpp
// This is the game loop
while (window.isOpen()) {
    // while the screen hasn't been closed

    sf::Event event;
    while (window.pollEvent(event)) {
        // process all events between the two gameticks
        switch (event.type) {
            case sf::Event::EventType::Closed:
            ...
        }
    }

    ImGui::SFML::Update(window, delta_clock_.restart());
    window.clear(); // clear the window
    ImGui::SFML::Render(window); // render ImGui stuff
    screen->draw(window); // draw stuff in the window
    window.display(); // display the window

}
```

## System Architecture
In order to achieve the the maintenance goals of the project, I tried to use design patterns whenever possible to mediate data-access and enforce certain behaviors. 

### State Design Pattern: Managing Behavior between Screens

To manage behavior of the app between "screens" (i.e. the play screen with the chess board or the setup screen with the options to play as white or as black), I used the state design pattern.

Sample code for the state design pattern may be found below. The main benefit of this pattern is that it __"lets an object alter its behavior when its internal state changes"__ [[1](#guru)].

In other words, after the user has clicked some buttons on the main splash screen, the app needs to transition to another screen with different graphics resources and different logic for how to respond to different user-inputs. One way of managing the state changes would be to have a member variable in the application class that holds the state information, then have something like a switch statement to handle the difference in logic. That approach might be okay for very simple state-diagrams, but as the transitioning logic becomes more complicated, the logic internal to a single state becomes more complex, and resource demands become increasingly heavy that paradigm becomes untenable.

The state design pattern keeps the transition logic simple, separates one state's logic from another state's logic, and decouples the state objects and state logic from that of the application, thereby massively simplifying the code.

```Cpp
class App; // forward declare the App

/** 
 * abstract state superclass that creates a prototype "Screen" class
 * and a consistent interface for callers
 */
class Screen {
protected:
    App* context_; // pointer to the application (context)

public:
    virtual ~Screen() = default;
    virtual void process_event(sf::Event& event) = 0;
    virtual void draw(sf::RenderWindow& window) = 0;

    void set_context(App* context) {
        this->context_ = context;
    }
}

/**
 * The application entry-point: manages our current state
 */
class App {
protected:
    friend class PlayScreen;
    friend class SplashScreen;
    friend class BattleSetupScreen;

private:
    std::unique_ptr<State> screen_;
    sf::RenderWindow window_;

    explicit LOTRChess(std::unique_ptr<Screen> init_screen) 
        : screen_(std::move(init_screen)) {
        screen_->set_context(this);
    }

    // transition to another screen
    void transition_to(Screen* screen) {
        screen_.reset(screen); // destroy old screen, store new one
        screen_->set_context(this); // set the context of screen to 'this'
    }
}

/**
 * concrete PlayScreen class inheriting from the Screen superclass.
 * handles logic and manages objects when the app is in the PlayScreen 
 * state.
 */
class PlayScreen : public Screen {
    explicit PlayScreen(Color player_color, Engine engine) {
        ...
    }

    ~PlayScreen {
        ...
    }

    /**
     * handles logic for this state
     */
    void process_event(sf::Event& event) override {
        ...
    }

    /**
     * handles logic for this state
     */
    void draw(sf::RenderWindow& window) override {
        ...
    }
}

/**
 * concrete SplashScreen class inheriting from the Screen superclass.
 * handles logic and manages objects when the app is in the SplashScreen 
 * state.
 */
class SplashScreen : public Screen {
    explicit SplashScreen() {
        ...
    }

    ~SplashScreen {
        ...
    }

    /**
     * handles logic for this state
     */
    void process_event(sf::Event& event) override {
        ...
    }

    /**
     * handles logic for this state
     */
    void draw(sf::RenderWindow& window) override {
        ...
    }
}
```

### Model-View-Controller (MVC): Managing Behavior of a Given Screen
To manage the user-input logic, the app's behavior, and the graphics of a single screen, I made use of the Model-View-Controller (MVC) paradigm.

Some sample code for the MVC design pattern may be found below. The main benefit of this paradigm is to __decouple the user-input logic, game state, and graphics from one another__. 

The central conceit of this design pattern is __separation of concerns__. The only thing that the View class should know about is how to draw the graphics resources. It should not know anything about how a click affects the layout nor should it know about whether the player is playing as white or black. 

Dido for the model. The model should not know about what graphics resources are being used nor should it know about how the user-input affects whether the player is black or white. It's only job is to hold onto the game state data.

__This allows the programmer to think about model, view, and controller code separately.__ Also, should I choose to change the game's view to be Star Wars-themed, then I won't affect the model or the controller code. Similarly, if I choose to add a feature allowing the user to use a joy-stick in addition to the mouse, then I just have to change the controller, but the model and view logic can remain untouched.

```cpp
/**
 * the concrete screen classes are effectively the controller objects
 */
class PlayScreen : public Screen {
private:
    PlayModel model_; // handles logic
    PlayView view_; // handles graphics

public:

    explicit PlayScreen(Color player_color, Engine engine) {
        ...
    }

    ~PlayScreen {
        ...
    }

    /**
     * the PlayScreen class manages how the user-input affects the model
     */
    void process_event(sf::Event& event) override {
        switch (event.type) {
            case sf::Event::Closed:
                context_->window_.close();
                break;
            case sf::Event::KeyPressed {
                if (event.key.code == sf::Keyboard::R) {
                    // if the user pressed 'R', then reverse the board
                    model_.rotate_board(); // update the model
                    view_.update(model_); // update view given model
                }
                ...
                } else if (event.key.code == sf::Keyboard::Escape) {
                    // if user pressed ESC, go back to the splash screen
                    context_->transition_to(new SplashScreen());
                }
            }
            ...
        }
    }

    /**
     * forwards the draw requests onto the view, which is suited to 
     * handle such things
     */
    void draw(sf::RenderWindow& window) override {
        view_.draw(window); // pass the window to the view
    }
}
```

## Multithreading
The application makes extensive use of multi-threading to improve its overall performance. For example, if it weren't for dedicated threads handling move generation and inter-process communication with the chess engine, then the application would constantly stall. 

However, as soon as multiple threads enter the picture, the potential for introducing a host of bugs related to __data-access__ and __resource ownership__ opens up. 

Nominally, mutexes for synchronizing data-access need to be used on any piece of mutable data being read or written to from more than one thread. The standard C++ library has tools for doing that in such a way as to avoid deadlocking (should they be deployed correctly), like ```std::lock_guard<std::mutex>```. 

```cpp
// checks to see whether the "quit_" member variable is true
bool EngineInterface::check_quit() {
    std::lock_guard<std::mutex> guard(mu_quit_);
    return quit_;
}

// sets the "quit_" member variable to true
void EngineInterface::quit() {
    std::lock_guard<std::mutex> guard(mu_quit_);
    quit_ = true;
}
```

But what's more important than simply using mutexes is having an architecture in place that enforces a certain type of communication between threads, so that a programmmer can reason more easily about the code and more quickly identify bugs arising from multithreading. 

One such architecture is uni-directional communication. In this case, the main thread makes requests to the engine I/O thread with no expectation of ever getting a result. The engine I/O thread does not know about the main thread at all, and simply services requests. Then, upon finishing the request, the engine I/O thread will update some  member variables that are visible to the main thread. The main thread will then perform a synchronized read of such member variables to get the result. 

The main benefits of uni-directional communication are two-fold. For one, the programmer can focus on implementing the engine I/O logic without being concerned about communicating with other threads. Second, the synchronization logic is entirely encapsulated in the EngineInterface class, so making requests to the EngineInterface class from the main-thread looks like making a function call. 


## Conclusion
My goal for this project was to build a chess UI that would enable me to play against different UCI chess engines. Though the game isn't perfect and there are still many features I wish to implement, the bare-bones of the app is complete!

Concurrent with that functional goal were non-functional goals related to the maintenance and efficiency of the code. By making deliberate choices about which design patterns to deploy as well as seeking to use multiple threads to handle different aspects of the project, I was able to achieve those non-functional goals as well.

### References
[1] <a href="https://refactoring.guru/design-patterns/state">https://refactoring.guru/design-patterns/state</a><a id="guru">