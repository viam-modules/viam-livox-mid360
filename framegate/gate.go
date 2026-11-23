// Package framegate decides when a reader may take the newest frame from a
// producer that replaces it in place.
package framegate

import (
	"context"
	"errors"
	"sync"
	"time"
)

// ErrTimeout is returned by Wait when no frame arrived in time.
var ErrTimeout = errors.New("timeout waiting for a frame")

// Gate lets every reader have each frame. A frame that nobody has read yet is
// given to the first reader at once, and a reader that finds the newest frame
// already taken waits for the next one together with everyone else who is
// waiting. So one slow reader still gets the newest frame without a wait, and
// several readers all see every frame instead of sharing them out between
// them.
type Gate struct {
	mu sync.Mutex
	// closed and replaced each time a frame is published, which wakes every
	// reader waiting on it
	next chan struct{}
	// true when a reader has already had the newest frame, and before the
	// first frame exists
	taken bool
}

// New returns a Gate with no frame published yet.
func New() *Gate {
	return &Gate{next: make(chan struct{}), taken: true}
}

// Publish says a new frame is ready to read. It never blocks on a reader.
func (g *Gate) Publish() {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.taken = false
	close(g.next)
	g.next = make(chan struct{})
}

// Wait returns nil when the caller may read the newest frame. It returns the
// context's error if the context ends first, and ErrTimeout if no frame was
// published within the timeout.
func (g *Gate) Wait(ctx context.Context, timeout time.Duration) error {
	g.mu.Lock()
	if !g.taken {
		g.taken = true
		g.mu.Unlock()
		return nil
	}
	next := g.next
	g.mu.Unlock()

	timer := time.NewTimer(timeout)
	defer timer.Stop()
	select {
	case <-next:
	case <-ctx.Done():
		return ctx.Err()
	case <-timer.C:
		return ErrTimeout
	}

	// we mark it taken so that a reader arriving after us waits for the next
	// frame and does not get this one a second time through the fast path
	g.mu.Lock()
	g.taken = true
	g.mu.Unlock()
	return nil
}
