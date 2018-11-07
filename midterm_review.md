# Midterm Review

## UML

### Use Case Diagram
**Actors** (stick figures)
**Use Cases** (circles with use cases)
Draw lines between actors and use cases

### Class Diagram
**Dependency** (Dotted line)
**Association** (Solid black line)
**Aggregation** (Outline of diamond w/ solid line)
**Composition** (Solid diamond. Solid line) (Object and class die together)

**Generalization** (Subclass/Inheritance) Triangle w/ solid line
**Realization** (Subtype/Interface) Triangle w/ dotted line

## Design Patterns

### Structural

#### Adaptor
     The Adapter Pattern converts the interface of a class into another interface the clients expect. 
     Adapter lets classes work together that couldn’t otherwise because of incompatible interfaces.

#### Facade
    The Façade Pattern provides a unified interface to a set of interfaces in a subsystem. 
    Façade defines a higher-level interface that makes the subsystem easier to use.

#### Flyweight
    The Flyweight Pattern allows one instance of a class can be reused to provide many virtual instances.
~~~
      for(int i=0; i < 20; ++i) {
         Circle circle = (Circle)ShapeFactory.getCircle(getRandomColor());
         circle.setX(getRandomX());
         circle.setY(getRandomY());
         circle.setRadius(100);
         circle.draw();         // This is the flyweight part. One object drawn 20 times
      }
~~~

### Creational

### Abstract factory
    The Abstract Factory Pattern provides an interface for creating families of related or dependent objects 
    without specifying their concrete classes.

Hence the word *interface*.
   
Delegates responsibility of **object** instantiatin to another object via composition

Pros
- Dynamic at runtime
Cons
- Adding an "ingredient" requires changing interface and all subtypes
~~~
    public interface abstractFactory() {
        createA();
        createB();
    }
    public A(Factory factory) {
        // The object passed into factory will determine what gets created
        this.factory = factory
    }
~~~
#### Factory 
    The Factory Method Pattern defines an interface for creating an object but lets subclasses decide which 
    class to instantiate. Factory Method lets a class defer instantiation to subclasses

"Factory Method lets a class defer instantiation to subclasses"
- In the case of the `PizzaStore`, `createPizza()`in each subclass handles instantiation

Pattern uses **inheritance** and relies on subclass to another desired object instantiation
- Not dynamic at runtime


#### Singleton
    The Singleton Pattern ensures a class has only one instance and provides a global point of access to 
    that instance.

### Beahavioral

#### Template
    The Template Method Pattern defines the skeleton of an algorithm in a method, deferring some steps to 
    subclasses. Template Method lets subclasses redefine certain steps of an algorithm without changing 
    the algorithm’s structure.

Look for 1+ abstract method that subclasses need to implement. 

#### Command
    The Command Pattern encapsulates a request as an object, thereby letting you parameterize other objects 
    with different requests, queue or log requests, and support undoable operations.

#### Observer
    The Observer Pattern defines a one-to- many dependency between objects so that when one object changes 
    state, all of its dependents are notified and updated automatically.

#### Stategy
    The Strategy Pattern defines a family of algorithms, Encapsulates each one, and makes them interchangeable 
    at runtime. Strategy lets the algorithm vary independently from clients that use it.

Usually associated with a behavior. `setBehavior()` is indication. 

#### State
    The State Pattern allows an object to alter its behavior when its internal state changes. The object will 
    appear to change its class.
    
Class for each state. 
- State interface will have every possible transition?
- Each state needs to implement every transition (I think)
An additional context state.

#### Mediator
    Define an object that encapsulates how a set of objects interact. Promotes loose coupling by keeping 
    objects from referring to each other explicitly and it lets you vary their interactions independently.
    Encapsulates many to many dependencies between objects

**Pros**
- Communication logic is simple
- Centralized control logic
**Cons** 
- Single point of vulnerability
- Complexity in mediator


### Composition
Composition is more flexible because to change behavior you simply set the field. Opposite would be subclassing. 
Not flexible because behaviors cannot be modified within objects. 


## Bad Code Smells

### Divergent Changes 
**Problem**: One class suffers many different changes
**Solution**: Extract Class

### Shotgun Surgery
**Problem**: One change alters many classes
**Solution**: Move method, field, inline class

### Feature Envy
**Problem**: Method more interested in a class other than the one it is actually in
**Solution**: Move method, Extract method, 

### Data Clumps
**Problem**: Bunches of data that hang around together really ought to be made into their own object
**Solution**: Extract class, preserve whole object, parameter objects

### Switch Statements 
When you see switch statements, you should think of polymorphism. UNLESS it is very simple

### Parallel inheritance hierarchies 
**Problem**: Every time you make a subclass of onse class, you also have to make a subclass of another. 
**Solution**: Move method or field

### Speculative Generality
**Problem**: Sometimes code is created "just in case" to support anticipated future features that never get implemented. As a result, code becomes hard to understand and support.
**Solution**: Literally just clean it up

### Inappropiate Intimacy
**Problem**: Keep a close eye on classes that spend too much time together. Good classes should know as little about each other as possible. Such classes are easier to maintain and reuse.
**Solution**: Move method/field, Hide Delegate (If this->getB()->Bmethod() just do this->Bmethod())









