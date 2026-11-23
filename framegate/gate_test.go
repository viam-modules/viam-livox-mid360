package framegate

import (
	"context"
	"errors"
	"sync"
	"sync/atomic"
	"testing"
	"time"
)

// A reader that asks again as soon as it has a frame, the way a polling
// client does. It counts the frames it was given.
func poll(ctx context.Context, g *Gate, got *atomic.Int64, wg *sync.WaitGroup) {
	defer wg.Done()
	for {
		if err := g.Wait(ctx, time.Second); err != nil {
			return
		}
		got.Add(1)
	}
}

func TestEveryReaderGetsEveryFrame(t *testing.T) {
	const frames = 50
	const readers = 3
	g := New()
	ctx, cancel := context.WithCancel(context.Background())
	var wg sync.WaitGroup
	counts := make([]atomic.Int64, readers)
	for i := range counts {
		wg.Add(1)
		go poll(ctx, g, &counts[i], &wg)
	}

	for i := 0; i < frames; i++ {
		// long enough for every reader to be waiting again before the next frame
		time.Sleep(2 * time.Millisecond)
		g.Publish()
	}
	time.Sleep(10 * time.Millisecond)
	cancel()
	wg.Wait()

	for i := range counts {
		// with the channel of size one this replaced, three readers got about
		// a third of the frames each
		if got := counts[i].Load(); got != frames {
			t.Errorf("reader %d got %d frames, want %d", i, got, frames)
		}
	}
}

func TestUnreadFrameIsGivenAtOnce(t *testing.T) {
	g := New()
	g.Publish()
	start := time.Now()
	if err := g.Wait(context.Background(), time.Second); err != nil {
		t.Fatalf("Wait: %v", err)
	}
	if waited := time.Since(start); waited > 50*time.Millisecond {
		t.Errorf("waited %v for a frame that was already there", waited)
	}
}

func TestFrameIsNotGivenTwice(t *testing.T) {
	g := New()
	g.Publish()
	if err := g.Wait(context.Background(), time.Second); err != nil {
		t.Fatalf("first Wait: %v", err)
	}
	err := g.Wait(context.Background(), 20*time.Millisecond)
	if !errors.Is(err, ErrTimeout) {
		t.Errorf("second Wait returned %v, want ErrTimeout", err)
	}
}

func TestWaitBeforeFirstFrame(t *testing.T) {
	g := New()
	if err := g.Wait(context.Background(), 20*time.Millisecond); !errors.Is(err, ErrTimeout) {
		t.Errorf("Wait returned %v with no frame published, want ErrTimeout", err)
	}
}

func TestWaitEndsWithContext(t *testing.T) {
	g := New()
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if err := g.Wait(ctx, time.Second); !errors.Is(err, context.Canceled) {
		t.Errorf("Wait returned %v, want context.Canceled", err)
	}
}
